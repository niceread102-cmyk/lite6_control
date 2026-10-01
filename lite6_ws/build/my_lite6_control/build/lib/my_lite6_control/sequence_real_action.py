import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
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


# Poses, durations, approach distances, planning scaling, gripper services and topic names:
# config/real.yaml (= config/default.yaml + the real cell's goal pose and timings).
REAL_CONFIG = 'real.yaml'
POSE_NAMES  = ('home', 'init', 'goal')


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


class SequenceRealNode(Node):
    def __init__(self, cfg=None):
        super().__init__('sequence_real_node')
        self.cfg          = cfg if cfg is not None else load_config(default_config_path(REAL_CONFIG))
        ros               = self.cfg.ros
        self._poses       = {name: named_pose(self.cfg, name) for name in POSE_NAMES}
        self._traj_client = ActionClient(self, FollowJointTrajectory, ros.trajectory_action)
        self.pose_sub     = self.create_subscription(Pose, ros.target_pose_topic, self._cb_target_pose, 10)
        self.joint_state_sub = self.create_subscription(JointState, ros.joint_states_topic, self._cb_joint_state, 10)
        self._client      = ActionClient(self, MoveGroup, ros.move_group_action)
        self._ik_client   = self.create_client(GetPositionIK, ros.compute_ik_service)
        self._state       = ControlState.INIT
        self._grasp_pose  = None
        self._pending_target = None
        self._busy        = False
        self.current_joint_state = None

        self.get_logger().info(describe(self.cfg))
        self._startup_timer = self.create_timer(self.cfg.sequence.start_delay_s, self._start_init)

    def _start_init(self):
        self._startup_timer.cancel()
        self.send_home(on_done=self._on_init_done)
        self.call_gripper(self.cfg.robot.gripper.open_service)

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

    def _cb_joint_state(self, msg: JointState):
        self.current_joint_state = msg

    def _cb_target_pose(self, msg: Pose):
        if self._busy:
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
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._state = ControlState.DIVE
        descend_dist = self.cfg.sequence.descend_dist_m
        descend_pose = pre_grasp_pose(self._pre_pose, dist=-descend_dist)
        self.get_logger().info(
            f'[DIVE] descending {descend_dist}m past grasp pose along approach axis '
            f'pos=({descend_pose.position.x:.3f}, {descend_pose.position.y:.3f}, {descend_pose.position.z:.3f})')
        self._move_to(descend_pose, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._state = ControlState.GRASP
        self._grasp_settle_timer = self.create_timer(self.cfg.sequence.grasp_delay_s, self._do_grasp)

    def _do_grasp(self):
        self._grasp_settle_timer.cancel()
        self.get_logger().info('[GRASP] closing gripper')
        self.call_gripper(self.cfg.robot.gripper.close_service)
        self._after_grasp()

    def _after_grasp(self):
        self._state = ControlState.LIFT
        self.get_logger().info('[LIFT] retreating to pre-grasp pose')
        self._move_to(self._pre_pose, self._on_lift_done)

    def _on_lift_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._return_home()

    def _return_home(self):
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning to init')
        self.send_init(on_done=self._on_home_arrived)

    def _on_home_arrived(self, ok):
        if not ok:
            self.get_logger().error('[HOME] failed to reach init pose')
        self._state = ControlState.GOAL
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
        self.call_gripper(self.cfg.robot.gripper.open_service)
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

    def _move_to(self, pose: Pose, on_done):
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
            lambda future: self._on_ik_result(future, on_done))

    def _on_ik_result(self, future, on_done):
        joint_targets, err = ik_joint_targets(future.result(), self.cfg)
        if err:
            self.get_logger().error(f'[FAIL] {err}')
            on_done(False)
            return

        if not self._client.wait_for_server(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('MoveGroup not available')
            on_done(False)
            return

        # plan_only=True, same as sim — moveit2 just plans, then we forward the
        # planned trajectory through _send_traj (same follow_joint_trajectory
        # action client used for HOME/INIT/GOAL) instead of letting MoveGroup's
        # own moveit_simple_controller_manager execute it internally.
        goal = build_joint_goal(joint_targets, self.cfg, self.cfg.sequence.velocity_scaling,
                                self.cfg.sequence.acceleration_scaling, plan_only=True)
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
            self.get_logger().error(
                f'[FAIL] error_code={result.error_code.val} message="{result.error_code.message}" '
                f'source="{result.error_code.source}"')
            on_done(False)
            return

        traj = result.planned_trajectory.joint_trajectory
        if not prepare_trajectory(traj, self.cfg, self.get_logger()):
            on_done(False)
            return
        traj.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'[OK] sending trajectory via follow_joint_trajectory action ({len(traj.points)} pts)')
        self._send_traj(traj, on_done)

    def _send_traj(self, jt: JointTrajectory, on_done=None):
        if not self._traj_client.wait_for_server(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('follow_joint_trajectory action server not available')
            if on_done:
                on_done(False)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        self._traj_client.send_goal_async(goal).add_done_callback(
            lambda future: self._traj_goal_cb(future, on_done))

    def _traj_goal_cb(self, future, on_done):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Trajectory goal rejected')
            if on_done:
                on_done(False)
            return
        handle.get_result_async().add_done_callback(
            lambda future: self._traj_result_cb(future, on_done))

    def _traj_result_cb(self, future, on_done):
        result = future.result().result
        ok = (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        if not ok:
            self.get_logger().error(f'[FAIL] Trajectory execution error_code={result.error_code}')
        if on_done:
            on_done(ok)

    def call_gripper(self, srv_name):
        gripper = self.cfg.robot.gripper
        subprocess.run(
            f'source {gripper.service_setup} && ros2 service call {srv_name} xarm_msgs/srv/Call "{{}}"',
            shell=True, executable='/bin/bash'
        )
        time.sleep(gripper.service_wait_s)


def main(args=None):
    cfg = load_cli_config(REAL_CONFIG)  # --config <yaml> / dotted overrides after the executable name
    rclpy.init(args=args)
    node = SequenceRealNode(cfg)
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
