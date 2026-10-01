import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from my_lite6_control.config import default_config_path, describe, load_cli_config, load_config, named_pose
from my_lite6_control.motion import (build_ik_request, build_joint_goal, eef_target_from_tcp, fixed_move_duration,
                                     ik_joint_targets, joint_trajectory, pose_is_safe, pre_grasp_pose,
                                     prepare_trajectory)
from enum import Enum, auto
import subprocess
import time


# Poses, durations, approach distances, planning scaling, arrival check and topic names:
# config/default.yaml (robot.poses, sequence, planning, ros). "Arrived" is decided by
# watching /joint_states ourselves (sequence.arrival), not by the controller's goal_time
# timing — HOME/INIT/GOAL_MID/GOAL fixed joint moves and _move_to (compute_ik result)
# moves all go through _send_traj, so this one check covers the whole pipeline.
POSE_NAMES = ('home', 'init', 'goal_mid', 'goal')


class ControlState(Enum):
    INIT     = auto()
    MOVE     = auto()
    TO_INIT  = auto()
    DIVE     = auto()
    GRASP    = auto()
    LIFT     = auto()
    HOME     = auto()
    GOAL     = auto()
    DROP     = auto()


class SequenceNode(Node):
    def __init__(self, cfg=None):
        super().__init__('sequence_node')
        self.cfg = cfg if cfg is not None else load_config(default_config_path())
        ros = self.cfg.ros
        self._poses = {name: named_pose(self.cfg, name) for name in POSE_NAMES}
        self._traj_client = ActionClient(self, FollowJointTrajectory, ros.trajectory_action)
        # Optional Isaac Sim mirror (ros.sim_mirror_action) — separate action, separate
        # name, on purpose. Fire-and-forget: only cosmetic, never gates sequence
        # progression, which is driven solely by the real controller via /joint_states
        # (_check_arrival below). sim/lite6_motion_sim.py does not need it: it follows
        # /joint_states itself.
        self._sim_traj_client = (ActionClient(self, FollowJointTrajectory, ros.sim_mirror_action)
                                 if ros.sim_mirror_action else None)
        self.gripper_pub  = self.create_publisher(Float32, ros.gripper_command_topic, 10)
        self.pose_sub     = self.create_subscription(Pose, ros.target_pose_topic, self._cb_target_pose, 10)
        self.joint_state_sub = self.create_subscription(JointState, ros.joint_states_topic, self._cb_joint_state, 10)
        self._client     = ActionClient(self, MoveGroup, ros.move_group_action)
        self._ik_client  = self.create_client(GetPositionIK, ros.compute_ik_service)
        self._state      = ControlState.INIT
        self._grasp_pose = None
        self._pending_target = None
        self._busy       = False
        self.current_joint_state = None

        self._arrival_on_done = None
        self._arrival_fired   = False
        self._arrival_timer   = None
        self._settle_timer    = None

        self.get_logger().info(describe(self.cfg))
        self._startup_timer = self.create_timer(self.cfg.sequence.start_delay_s, self._start_init)

    def _start_init(self):
        self._startup_timer.cancel()
        self.send_home(on_done=self._on_init_done)
        self.open_gripper()

    def _on_init_done(self, ok):
        if not ok:
            self.get_logger().error('[INIT] failed to reach home pose')
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def _send_joint_positions(self, pose_name, move_sec, on_done):
        positions = self._poses[pose_name]
        requested = self.cfg.sequence.move_time_s[pose_name] if move_sec is None else move_sec
        duration  = fixed_move_duration(positions, self.current_joint_state, self.cfg, requested)
        if duration > requested + 1e-6:
            self.get_logger().info(f'[LIMIT] {pose_name} move lengthened {requested:.2f} -> {duration:.2f} s '
                                   f'(joint max velocity)')
        jt = joint_trajectory(self.cfg, positions, duration, self.get_clock().now().to_msg())
        self._send_traj(jt, on_done)

    def send_home(self, move_sec=None, on_done=None):
        self.get_logger().info('home')
        self._send_joint_positions('home', move_sec, on_done)

    def send_init(self, move_sec=None, on_done=None):
        self.get_logger().info('init')
        self._send_joint_positions('init', move_sec, on_done)

    def send_goal_mid(self, move_sec=None, on_done=None):
        self.get_logger().info('goal (mid)')
        self._send_joint_positions('goal_mid', move_sec, on_done)

    def _cb_joint_state(self, msg: JointState):
        self.current_joint_state = msg

    def _cb_target_pose(self, msg: Pose):
        if self._busy or self._state != ControlState.MOVE:
            self.get_logger().warn(f'Busy (state={self._state.name}), ignoring target until current cycle finishes')
            return

        if not pose_is_safe(msg, self.get_logger(), self.cfg):
            return

        self._busy = True
        self._pending_target = msg
        self._state = ControlState.TO_INIT
        self.get_logger().info('[TO_INIT] re-homing to init pose before approaching target')
        self.send_init(on_done=self._on_to_init_done)

    def _on_to_init_done(self, ok):
        self._state = ControlState.MOVE
        if not ok:
            self.get_logger().error('[TO_INIT] failed to reach init pose')
            self._busy = False
            return

        msg = self._pending_target
        self._grasp_pose = msg
        self._pre_pose = pre_grasp_pose(msg, self.cfg.sequence.approach_dist_m)
        self.get_logger().info(f'[MOVE] pre-grasp pos=({self._pre_pose.position.x:.3f}, {self._pre_pose.position.y:.3f}, {self._pre_pose.position.z:.3f})')
        self._move_to(self._pre_pose, self._on_pre_grasp_done)

    def _on_pre_grasp_done(self, ok):
        if not ok:
            self._abort_to_ready('pre-grasp move failed/rejected')
            return

        self._state = ControlState.DIVE
        descend_dist = self.cfg.sequence.descend_dist_m
        descend_pose = pre_grasp_pose(self._pre_pose, dist=-descend_dist)
        self.get_logger().info(
            f'[DIVE] descending {descend_dist}m from pre-grasp toward grasp pose '
            f'pos=({descend_pose.position.x:.3f}, {descend_pose.position.y:.3f}, {descend_pose.position.z:.3f})')
        self._move_to(descend_pose, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._abort_to_ready('descend move failed/rejected')
            return

        self._state = ControlState.GRASP
        self._grasp_pre_settle_timer = self.create_timer(self.cfg.sequence.grasp_delay_s, self._do_grasp)

    def _do_grasp(self):
        self._grasp_pre_settle_timer.cancel()
        self.get_logger().info('[GRASP] closing gripper')
        self.close_gripper()
        self._grasp_settle_timer = self.create_timer(self.cfg.sequence.grasp_hold_s, self._after_grasp)

    def _after_grasp(self):
        self._grasp_settle_timer.cancel()
        self._state = ControlState.LIFT
        self.get_logger().info('[LIFT] retreating to pre-grasp pose')
        self._move_to(self._pre_pose, self._on_lift_done)

    def _on_lift_done(self, ok):
        if not ok:
            # GRASP already ran before LIFT was attempted, so the gripper may
            # still be holding the object here — release it before recovering.
            self._abort_to_ready('lift move failed/rejected after grasping', had_object=True)
            return

        self._return_home()

    def _abort_to_ready(self, reason, had_object=False):
        """Bail out of the current pickup cycle and get back to the idle MOVE
        state so the next queued target (auto_pick_loop_sim's VisionWatcher)
        can be sent — used when a _move_to() planning/execution call fails,
        e.g. the obstacle-avoiding planner can't find a valid trajectory to
        this target. The failed target itself is simply dropped (VisionWatcher
        already popped it off the queue before publishing); we do not retry
        it here."""
        self.get_logger().warn(f'[ABORT] {reason} -> recovering to HOME, moving on to next target')
        if had_object:
            self.get_logger().warn('[ABORT] gripper was holding the object mid-cycle, opening to release it safely')
            self.open_gripper()

        self._state = ControlState.HOME
        self.send_home(on_done=self._on_abort_recovered)

    def _on_abort_recovered(self, ok):
        if not ok:
            self.get_logger().error('[ABORT] failed to recover to home pose')
        self._state = ControlState.MOVE
        self._busy = False
        self.get_logger().info('[MOVE] recovered from abort, waiting for target pose')

    def _return_home(self):
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning to init')
        self.send_init(on_done=self._on_home_arrived)

    def _on_home_arrived(self, ok):
        if not ok:
            # Object is already grasped and lifted here too — don't push on to
            # GOAL_MID from an unverified position, release and recover instead.
            self._abort_to_ready('failed to reach init pose while returning with object', had_object=True)
            return
        self._state = ControlState.GOAL
        self.get_logger().info('[GOAL] moving to intermediate goal pose')
        self.send_goal_mid(on_done=self._on_goal_mid_arrived)

    def _on_goal_mid_arrived(self, ok):
        if not ok:
            # Object is already grasped and lifted at this point — same
            # had_object handling as a failed LIFT: release it safely instead
            # of leaving it clamped, and (critically) actually release _busy
            # so the pipeline doesn't stall forever waiting for a target that
            # will never come.
            self._abort_to_ready('failed to reach intermediate goal pose', had_object=True)
            return

        self.get_logger().info('[GOAL] moving to goal pose')
        self.send_goal(on_done=self._on_goal_arrived)

    def send_goal(self, move_sec=None, on_done=None):
        self.get_logger().info('goal')
        self._send_joint_positions('goal', move_sec, on_done)

    def _on_goal_arrived(self, ok):
        if not ok:
            self.get_logger().error('[GOAL] failed to reach goal pose')
        self._state = ControlState.DROP
        self.get_logger().info('[DROP] opening gripper')
        self.open_gripper()
        self._drop_timer = self.create_timer(self.cfg.sequence.drop_wait_s, self._return_home_final)

    def _return_home_final(self):
        self._drop_timer.cancel()
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning home')
        self.send_home(on_done=self._on_cycle_done)

    def _on_cycle_done(self, ok):
        if not ok:
            self.get_logger().error('[HOME] failed to reach home pose')
        self._state = ControlState.MOVE
        self._busy = False
        self.get_logger().info('[MOVE] waiting for target pose')

    def _move_to(self, pose: Pose, on_done, vel_scale=None, acc_scale=None):
        if vel_scale is None:
            vel_scale = self.cfg.sequence.velocity_scaling
        if acc_scale is None:
            acc_scale = self.cfg.sequence.acceleration_scaling

        if self.current_joint_state is None:
            self.get_logger().error('No /joint_states yet, cannot seed IK')
            on_done(False)
            return

        if not self._ik_client.wait_for_service(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('compute_ik service not available')
            on_done(False)
            return

        target = eef_target_from_tcp(pose, self.cfg)
        ik_req = build_ik_request(target, self.current_joint_state, self.cfg)
        self._ik_client.call_async(ik_req).add_done_callback(
            lambda future: self._on_ik_result(future, on_done, vel_scale, acc_scale))

    def _on_ik_result(self, future, on_done, vel_scale, acc_scale):
        joint_targets, err = ik_joint_targets(future.result(), self.cfg)
        if err:
            self.get_logger().error(f'[FAIL] {err}')
            on_done(False)
            return

        if not self._client.wait_for_server(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('MoveGroup not available')
            on_done(False)
            return

        # sim keeps plan_only=True — it manually forwards the planned trajectory
        # through its own follow_joint_trajectory action client (see _result_cb)
        # rather than letting MoveGroup execute it directly.
        goal = build_joint_goal(joint_targets, self.cfg, vel_scale, acc_scale, plan_only=True)
        self._client.send_goal_async(goal).add_done_callback(
            lambda future: self._goal_cb(future, on_done))

    def _goal_cb(self, future, on_done):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            on_done(False)
            return
        handle.get_result_async().add_done_callback(
            lambda future: self._result_cb(future, on_done))

    def _result_cb(self, future, on_done):
        result = future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(f'[FAIL] error_code={result.error_code.val}')
            on_done(False)
            return

        traj = result.planned_trajectory.joint_trajectory
        if not prepare_trajectory(traj, self.cfg, self.get_logger()):
            on_done(False)
            return
        traj.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'[OK] sending trajectory via follow_joint_trajectory action ({len(traj.points)} pts)')
        self._send_traj(traj, on_done)

    def _send_sim_traj(self, jt: JointTrajectory):
        # Best-effort mirror to Isaac Sim — short timeout, no result tracking.
        # If the sim bridge isn't up, the real robot/RViz still work fine.
        if self._sim_traj_client is None:
            return
        if not self._sim_traj_client.wait_for_server(timeout_sec=self.cfg.ros.sim_mirror_timeout_s):
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        self._sim_traj_client.send_goal_async(goal)

    def _send_traj(self, jt: JointTrajectory, on_done=None):
        self._send_sim_traj(jt)

        if not self._traj_client.wait_for_server(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('follow_joint_trajectory action server not available')
            if on_done:
                on_done(False)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        target = list(jt.points[-1].positions)
        self._traj_client.send_goal_async(goal).add_done_callback(
            lambda future: self._traj_goal_cb(future, target, on_done))

    def _traj_goal_cb(self, future, target, on_done):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Trajectory goal rejected')
            if on_done:
                on_done(False)
            return

        # Don't wait for the action result — completion is decided by the polling
        # below (_check_arrival) straight from /joint_states. The action result is
        # still subscribed, only to fail fast on errors (_traj_result_cb).
        self._arrival_on_done  = on_done
        self._arrival_fired    = False
        self._arrival_deadline = time.monotonic() + self.cfg.sequence.arrival.timeout_s
        self._arrival_timer    = self.create_timer(0.05, lambda: self._check_arrival(target))

        handle.get_result_async().add_done_callback(self._traj_result_cb)

    def _check_arrival(self, target):
        if self._arrival_fired or self.current_joint_state is None:
            return

        arrival = self.cfg.sequence.arrival
        cur = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        try:
            max_diff = max(abs(cur[n] - t) for n, t in zip(self.cfg.robot.joint_names, target))
        except KeyError:
            return  # joint names don't match yet (e.g. before the first full /joint_states) -> keep waiting

        if max_diff < arrival.threshold_rad:
            self._arrival_timer.cancel()
            self._settle_timer = self.create_timer(arrival.settle_s, lambda: self._finish_arrival(True))
            return

        if time.monotonic() > self._arrival_deadline:
            self._arrival_timer.cancel()
            self.get_logger().error(
                f'[TIMEOUT] joints did not converge within {arrival.timeout_s}s '
                f'(max_diff={max_diff:.4f} >= {arrival.threshold_rad})')
            self._finish_arrival(False)

    def _finish_arrival(self, ok):
        if self._arrival_fired:
            return
        self._arrival_fired = True
        if self._arrival_timer is not None:
            self._arrival_timer.cancel()
        if self._settle_timer is not None:
            self._settle_timer.cancel()
        on_done = self._arrival_on_done
        if on_done:
            on_done(ok)

    def _traj_result_cb(self, future):
        result = future.result().result
        ok = (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        if not ok:
            self.get_logger().error(f'[FAIL] Trajectory execution error_code={result.error_code}')
            self._finish_arrival(False)

    def call_gripper(self, srv_name):
        gripper = self.cfg.robot.gripper
        subprocess.run(
            f'source {gripper.service_setup} && ros2 service call {srv_name} xarm_msgs/srv/Call "{{}}"',
            shell=True, executable='/bin/bash'
        )
        time.sleep(gripper.service_wait_s)

    def open_gripper(self):
        self.gripper_pub.publish(Float32(data=float(self.cfg.robot.gripper.open_m)))
        self.get_logger().info('gripper open')

    def close_gripper(self):
        self.gripper_pub.publish(Float32(data=float(self.cfg.robot.gripper.close_m)))
        self.get_logger().info('gripper close')


def main(args=None):
    cfg = load_cli_config()  # --config <yaml> / dotted overrides after the executable name
    rclpy.init(args=args)
    node = SequenceNode(cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
