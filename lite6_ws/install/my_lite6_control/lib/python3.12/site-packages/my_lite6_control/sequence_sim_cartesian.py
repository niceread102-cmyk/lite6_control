"""sequence_sim.py + Cartesian path 실행 방식.

sequence_sim.py는 자유공간 OMPL 플래닝(ActionClient(MoveGroup))을 써서
approach->descend 사이 경로 형태가 보장 안 되고 자세가 흔들릴 수 있음.
이 버전은 moveit2_cartesian.py와 같은 방식(GetCartesianPath 서비스)으로
매 _move_to 호출을 직선 웨이포인트 보간으로 바꿔서 그 흔들림을 없앰.
상태 머신(TO_INIT/DIVE/GRASP/LIFT/HOME/GOAL/DROP) 로직은 sequence_sim.py와 동일.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32
from moveit_msgs.srv import GetCartesianPath
from scipy.spatial.transform import Rotation
from enum import Enum, auto
import numpy as np
import subprocess
import time


EEF_LINK      = 'link_eef'
BASE_FRAME    = 'link_base'
TCP_OFFSET    = 0.0836
MAX_STEP      = 0.005   # 웨이포인트 사이 최대 간격(m) - 작을수록 촘촘한 직선 보간
MIN_FRACTION  = 0.99    # 이보다 덜 완주하면 실패 처리

HOME          = [1.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 1.0]
INIT          = [0.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 0.0]
GOAL          = [-np.pi / 2, np.pi / 2, np.pi, 0.0, np.pi / 2, -np.pi / 2]
JOINT_NAMES   = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_OPEN  = 0.0089
GRIPPER_CLOSE = 0.0
APPROACH_DIST = 0.04
DESCEND_DIST  = 0.04
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


def build_cartesian_request(pose: Pose, vel_scale=VEL_SCALE, acc_scale=ACC_SCALE):
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = Pose()
    target.position.x, target.position.y, target.position.z = eef_pos.tolist()
    target.orientation = pose.orientation

    req                                  = GetCartesianPath.Request()
    req.header.frame_id                  = BASE_FRAME
    req.start_state.is_diff              = True
    req.group_name                       = 'lite6'
    req.link_name                        = EEF_LINK
    req.waypoints                        = [target]
    req.max_step                         = MAX_STEP
    req.jump_threshold                   = 0.0
    req.avoid_collisions                 = True
    req.max_velocity_scaling_factor      = vel_scale
    req.max_acceleration_scaling_factor  = acc_scale
    return req


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
    def __init__(self):
        super().__init__('sequence_node')
        self.traj_pub     = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.gripper_pub  = self.create_publisher(Float32, '/gripper/command', 10)
        self.pose_sub     = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self._client      = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._state       = ControlState.INIT
        self._grasp_pose  = None
        self._pending_target = None
        self._busy        = False

        self._startup_timer = self.create_timer(0.5, self._start_init)

    def _start_init(self):
        self._startup_timer.cancel()
        move_sec = 4.0
        self.send_home(move_sec)
        self.open_gripper()
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
        if self._busy:
            self.get_logger().warn(f'Busy (state={self._state.name}), ignoring target until current cycle finishes')
            return

        if not pose_is_safe(msg, self.get_logger()):
            return

        self._busy = True
        self._pending_target = msg
        self._state = ControlState.TO_INIT
        move_sec = 4.0
        self.get_logger().info('[TO_INIT] re-homing to init pose before approaching target')
        self.send_init(move_sec)
        self._to_init_timer = self.create_timer(move_sec + 1.0, self._on_to_init_done)

    def _on_to_init_done(self):
        self._to_init_timer.cancel()
        self._state = ControlState.MOVE

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
            f'[DIVE] descending {DESCEND_DIST}m from pre-grasp toward grasp pose '
            f'pos=({descend_pose.position.x:.3f}, {descend_pose.position.y:.3f}, {descend_pose.position.z:.3f})')
        self._move_to(descend_pose, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            self._busy = False
            return

        self._state = ControlState.GRASP
        self.get_logger().info('[GRASP] closing gripper')
        self.close_gripper()
        self._grasp_settle_timer = self.create_timer(1.5, self._after_grasp)

    def _after_grasp(self):
        self._grasp_settle_timer.cancel()
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
        self.open_gripper()
        self._drop_timer = self.create_timer(1.5, self._return_home_final)

    def _return_home_final(self):
        self._drop_timer.cancel()
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning home')
        self.send_home()
        self._final_home_timer = self.create_timer(5.0, self._on_cycle_done)

    def _on_cycle_done(self):
        self._final_home_timer.cancel()
        self._state = ControlState.MOVE
        self._busy = False
        self.get_logger().info('[MOVE] waiting for target pose')

    def _move_to(self, pose: Pose, on_done, vel_scale=VEL_SCALE, acc_scale=ACC_SCALE):
        if not self._client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('compute_cartesian_path service not available')
            on_done(False)
            return

        req = build_cartesian_request(pose, vel_scale, acc_scale)
        self._client.call_async(req).add_done_callback(
            lambda future: self._response_cb(future, on_done))

    def _response_cb(self, future, on_done):
        result = future.result()
        if result is None:
            self.get_logger().error('[FAIL] compute_cartesian_path call failed')
            on_done(False)
            return

        fraction = result.fraction
        if fraction < MIN_FRACTION:
            self.get_logger().error(
                f'[FAIL] Cartesian path only {fraction:.2%} complete '
                f'(straight line blocked by limit/collision/singularity), aborting')
            on_done(False)
            return

        traj = result.solution.joint_trajectory
        traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub.publish(traj)
        self.get_logger().info(f'[OK] Cartesian trajectory published (fraction={fraction:.2%}, {len(traj.points)} pts)')

        if traj.points:
            t = traj.points[-1].time_from_start
            delay = t.sec + t.nanosec * 1e-9 + 0.5
        else:
            delay = 1.0
        self._exec_wait_timer = self.create_timer(delay, lambda: self._on_traj_exec_done(on_done))

    def _on_traj_exec_done(self, on_done):
        self._exec_wait_timer.cancel()
        on_done(True)

    def call_gripper(self, srv_name):
        subprocess.run(
            f'source /home/v/xarm_ros2/install/setup.bash && ros2 service call {srv_name} xarm_msgs/srv/Call "{{}}"',
            shell=True, executable='/bin/bash'
        )
        time.sleep(1.5)

    def open_gripper(self):
        self.gripper_pub.publish(Float32(data=GRIPPER_OPEN))
        self.get_logger().info('gripper open')

    def close_gripper(self):
        self.gripper_pub.publish(Float32(data=GRIPPER_CLOSE))
        self.get_logger().info('gripper close')


def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()
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
