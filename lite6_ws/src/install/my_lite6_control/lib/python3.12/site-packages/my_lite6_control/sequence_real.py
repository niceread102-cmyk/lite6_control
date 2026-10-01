import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from enum import Enum, auto
from scipy.spatial.transform import Rotation
import numpy as np
import subprocess
import time


EEF_LINK      = 'link_eef'
BASE_FRAME    = 'link_base'
TCP_OFFSET    = 0.0836
HOME          = [1.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 1.0]
INIT          = [0.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 0.0]
GOAL          = [-np.pi / 2, np.pi / 2, np.pi, 0.0, np.pi / 4, 0.0]
JOINT_NAMES   = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
APPROACH_DIST = 0.005
DESCEND_DIST  = 0.005
VEL_SCALE     = 0.06
ACC_SCALE     = 0.02

WS_X       = (0.05, 0.45)
WS_Y       = (-0.45, 0.45)
WS_Z       = (0.01, 0.60)
MAX_REACH  = 0.45


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


def build_move_goal(pose: Pose, vel_scale=VEL_SCALE, acc_scale=ACC_SCALE):
    R = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    target.pose.position.x, target.pose.position.y, target.pose.position.z = eef_pos.tolist()
    target.pose.orientation = pose.orientation

    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
    bv     = BoundingVolume(primitives=[sphere], primitive_poses=[target.pose])

    pos_con                   = PositionConstraint()
    pos_con.header            = target.header
    pos_con.link_name         = EEF_LINK
    pos_con.constraint_region = bv
    pos_con.weight            = 1.0

    ori_con                             = OrientationConstraint()
    ori_con.header                      = target.header
    ori_con.link_name                   = EEF_LINK
    ori_con.orientation                 = target.pose.orientation
    ori_con.absolute_x_axis_tolerance   = 0.05
    ori_con.absolute_y_axis_tolerance   = 0.05
    ori_con.absolute_z_axis_tolerance   = 0.05
    ori_con.weight                      = 1.0

    req                                 = MotionPlanRequest()
    req.group_name                      = 'lite6'
    req.planner_id                      = 'RRTConnect'
    req.goal_constraints                = [Constraints(
        position_constraints    = [pos_con],
        orientation_constraints = [ori_con],
    )]
    req.num_planning_attempts           = 10
    req.allowed_planning_time           = 5.0
    req.max_velocity_scaling_factor     = vel_scale
    req.max_acceleration_scaling_factor = acc_scale

    goal_msg                             = MoveGroup.Goal()
    goal_msg.request                     = req
    goal_msg.planning_options.plan_only  = False
    return goal_msg


class ControlState(Enum):
    INIT  = auto()
    MOVE  = auto()
    DIVE  = auto()
    GRASP = auto()
    LIFT  = auto()
    HOME  = auto()
    GOAL  = auto()
    DROP  = auto()


class SequenceRealNode(Node):
    def __init__(self):
        super().__init__('sequence_real_node')
        self.traj_pub     = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.pose_sub     = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self._client      = ActionClient(self, MoveGroup, '/move_action')
        self._state       = ControlState.INIT
        self._grasp_pose  = None

        self._startup_timer = self.create_timer(0.5, self._start_init)

    def _start_init(self):
        self._startup_timer.cancel()
        move_sec = 4.0
        self.send_home(move_sec)
        self.call_gripper('/ufactory/open_lite6_gripper')
        self._init_timer = self.create_timer(move_sec + 1.0, self._on_init_done)

    def _on_init_done(self):
        self._init_timer.cancel()
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def send_home(self, move_sec=4.0):
        pt = JointTrajectoryPoint()
        pt.positions       = HOME
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info('home')

    def send_init(self, move_sec=4.0):
        pt = JointTrajectoryPoint()
        pt.positions       = INIT
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info('init')

    def _cb_target_pose(self, msg: Pose):
        if self._state != ControlState.MOVE:
            self.get_logger().warn(f'State={self._state.name}, ignoring')
            return

        if not pose_is_safe(msg, self.get_logger()):
            return

        self._grasp_pose = msg
        self._pre_pose = pre_grasp_pose(msg)
        self.get_logger().info(f'[MOVE] pre-grasp pos=({self._pre_pose.position.x:.3f}, {self._pre_pose.position.y:.3f}, {self._pre_pose.position.z:.3f})')
        self._move_to(self._pre_pose, self._on_pre_grasp_done)

    def _on_pre_grasp_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            return

        self._state = ControlState.DIVE
        descend_pose = pre_grasp_pose(self._grasp_pose, dist=-DESCEND_DIST)
        self.get_logger().info(
            f'[DIVE] descending {DESCEND_DIST}m past grasp pose along approach axis '
            f'pos=({descend_pose.position.x:.3f}, {descend_pose.position.y:.3f}, {descend_pose.position.z:.3f})')
        self._move_to(descend_pose, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            return

        self._state = ControlState.GRASP
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
            return

        self._return_home()

    def _return_home(self):
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning to init')
        self.send_init()
        self._home_arrive_timer = self.create_timer(5.0, self._on_home_arrived)

    def _on_home_arrived(self):
        self._home_arrive_timer.cancel()
        self._state = ControlState.GOAL
        self.get_logger().info('[GOAL] moving to goal pose')
        move_sec = 8.0
        self.send_goal(move_sec)
        self._goal_arrive_timer = self.create_timer(move_sec + 1.0, self._on_goal_arrived)

    def send_goal(self, move_sec=4.0):
        pt = JointTrajectoryPoint()
        pt.positions       = GOAL
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info('goal')

    def _on_goal_arrived(self):
        self._goal_arrive_timer.cancel()
        self._state = ControlState.DROP
        self.get_logger().info('[DROP] opening gripper')
        self.call_gripper('/ufactory/open_lite6_gripper')
        self._drop_timer = self.create_timer(1.0, self._return_home_final)

    def _return_home_final(self):
        self._drop_timer.cancel()
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning home')
        self.send_home()
        self._final_home_timer = self.create_timer(5.0, self._on_cycle_done)

    def _on_cycle_done(self):
        self._final_home_timer.cancel()
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def _move_to(self, pose: Pose, on_done):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('MoveGroup not available')
            on_done(False)
            return

        goal = build_move_goal(pose)
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
