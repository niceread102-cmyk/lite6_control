import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from my_lite6_control.moveit2_action import eef_target_from_tcp, build_ik_request, build_joint_goal
from enum import Enum, auto
from scipy.spatial.transform import Rotation
import numpy as np
import subprocess
import time


HOME          = [1.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 1.0]
INIT          = [0.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 0.0]
GOAL          = [-np.pi / 2, np.pi / 2, np.pi, 0.0, np.pi / 2, -np.pi / 2]
JOINT_NAMES   = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
APPROACH_DIST = 0.04
DESCEND_DIST  = 0.04
VEL_SCALE     = 0.06
ACC_SCALE     = 0.02

WS_X       = (0.05, 0.45)
WS_Y       = (-0.45, 0.45)
WS_Z       = (0.01, 0.60)
MAX_REACH  = 0.44


def pose_is_safe(msg: Pose, logger):
    p = np.array([msg.position.x, msg.position.y, msg.position.z])

    if not np.all(np.isfinite(p)):
        logger.error(f'target has NaN/Inf, DENIED: {p}')
        return False

    if not (WS_X[0] <= p[0] <= WS_X[1] and
            WS_Y[0] <= p[1] <= WS_Y[1] and
            WS_Z[0] <= p[2] <= WS_Z[1]):
        logger.error(f'target {np.round(p, 3)} outside workspace bounds '
                     f'x{WS_X} y{WS_Y} z{WS_Z}, DENIED')
        return False

    reach = float(np.linalg.norm(p))
    if reach > MAX_REACH:
        logger.error(f'target {np.round(p, 3)} reach={reach:.3f}m > {MAX_REACH}m, DENIED')
        return False

    return True


def pre_grasp_pose(msg: Pose, dist=APPROACH_DIST) -> Pose:
    R = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w]).as_matrix()
    approach_dir = R[:, 2]
    pos = np.array([msg.position.x, msg.position.y, msg.position.z]) - dist * approach_dir

    pre = Pose()
    pre.position.x, pre.position.y, pre.position.z = pos.tolist()
    pre.orientation = msg.orientation
    return pre


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
    def __init__(self):
        super().__init__('sequence_real_node')
        self._traj_client = ActionClient(self, FollowJointTrajectory,
                                          '/lite6_traj_controller/follow_joint_trajectory')
        self.pose_sub     = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._cb_joint_state, 10)
        self._client      = ActionClient(self, MoveGroup, '/move_action')
        self._ik_client   = self.create_client(GetPositionIK, '/compute_ik')
        self._state       = ControlState.INIT
        self._grasp_pose  = None
        self._pending_target = None
        self._busy        = False
        self.current_joint_state = None

        self._startup_timer = self.create_timer(0.5, self._start_init)

    def _start_init(self):
        self._startup_timer.cancel()
        self.send_home(4.0, on_done=self._on_init_done)
        self.call_gripper('/ufactory/open_lite6_gripper')

    def _on_init_done(self, ok):
        if not ok:
            self.get_logger().error('[INIT] failed to reach home pose')
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def send_home(self, move_sec=4.0, on_done=None):
        pt = JointTrajectoryPoint()
        pt.positions       = HOME
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.get_logger().info('home')
        self._send_traj(jt, on_done)

    def send_init(self, move_sec=4.0, on_done=None):
        pt = JointTrajectoryPoint()
        pt.positions       = INIT
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.get_logger().info('init')
        self._send_traj(jt, on_done)

    def _cb_joint_state(self, msg: JointState):
        self.current_joint_state = msg

    def _cb_target_pose(self, msg: Pose):
        if self._busy:
            self.get_logger().warn(f'Busy (state={self._state.name}), ignoring target until current cycle finishes')
            return

        if not pose_is_safe(msg, self.get_logger()):
            return

        self._busy = True
        self._pending_target = msg
        self._state = ControlState.TO_INIT
        self.get_logger().info('[TO_INIT] re-homing to init pose before approaching target')
        self.send_init(4.0, on_done=self._on_to_init_done)

    def _on_to_init_done(self, ok):
        self._state = ControlState.MOVE
        if not ok:
            self.get_logger().error('[TO_INIT] failed to reach init pose')
            self._busy = False
            return

        msg = self._pending_target
        self._grasp_pose = msg
        self._pre_pose = pre_grasp_pose(msg)
        self.get_logger().info(f'[MOVE] pre-grasp pos=({self._pre_pose.position.x:.3f}, {self._pre_pose.position.y:.3f}, {self._pre_pose.position.z:.3f})')
        self._move_to(self._pre_pose, self._on_pre_grasp_done)

    def _on_pre_grasp_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._state = ControlState.DIVE
        descend_pose = pre_grasp_pose(self._pre_pose, dist=-DESCEND_DIST)
        self.get_logger().info(
            f'[DIVE] descending {DESCEND_DIST}m past grasp pose along approach axis '
            f'pos=({descend_pose.position.x:.3f}, {descend_pose.position.y:.3f}, {descend_pose.position.z:.3f})')
        self._move_to(descend_pose, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._state = ControlState.GRASP
        self._grasp_settle_timer = self.create_timer(0.3, self._do_grasp)

    def _do_grasp(self):
        self._grasp_settle_timer.cancel()
        self.get_logger().info('[GRASP] closing gripper')
        self.call_gripper('/ufactory/close_lite6_gripper')
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
        self.send_goal(8.0, on_done=self._on_goal_arrived)

    def send_goal(self, move_sec=4.0, on_done=None):
        pt = JointTrajectoryPoint()
        pt.positions       = GOAL
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.get_logger().info('goal')
        self._send_traj(jt, on_done)

    def _on_goal_arrived(self, ok):
        if not ok:
            self.get_logger().error('[GOAL] failed to reach goal pose')
        self._state = ControlState.DROP
        self.get_logger().info('[DROP] opening gripper')
        self.call_gripper('/ufactory/open_lite6_gripper')
        self._drop_timer = self.create_timer(1.0, self._return_home_final)

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

        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('compute_ik service not available')
            on_done(False)
            return

        target = eef_target_from_tcp(pose)
        ik_req = build_ik_request(target, self.current_joint_state)
        self._ik_client.call_async(ik_req).add_done_callback(
            lambda future: self._on_ik_result(future, on_done))

    def _on_ik_result(self, future, on_done):
        response = future.result()
        if response.error_code.val != 1:
            self.get_logger().error(f'[FAIL] compute_ik error_code={response.error_code.val}')
            on_done(False)
            return

        solved = dict(zip(response.solution.joint_state.name,
                           response.solution.joint_state.position))
        try:
            joint_targets = [solved[n] for n in JOINT_NAMES]
        except KeyError:
            self.get_logger().error('IK solution missing expected joints')
            on_done(False)
            return

        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('MoveGroup not available')
            on_done(False)
            return

        # real executes immediately — no manual follow_joint_trajectory forwarding
        # (unlike sim's _on_ik_result), MoveGroup runs the trajectory itself.
        goal = build_joint_goal(joint_targets, VEL_SCALE, ACC_SCALE, plan_only=False)
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

        on_done(True)

    def _send_traj(self, jt: JointTrajectory, on_done=None):
        if not self._traj_client.wait_for_server(timeout_sec=3.0):
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
        subprocess.run(
            f'source /home/v/xarm_ros2/install/setup.bash && ros2 service call {srv_name} xarm_msgs/srv/Call "{{}}"',
            shell=True, executable='/bin/bash'
        )
        time.sleep(1.5)


def main(args=None):
    rclpy.init(args=args)
    node = SequenceRealNode()
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
