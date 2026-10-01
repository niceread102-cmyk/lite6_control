import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetCartesianPath
from scipy.spatial.transform import Rotation
import numpy as np

EEF_LINK    = 'link_eef'
BASE_FRAME  = 'link_base'
TCP_OFFSET  = 0.0836
MAX_STEP    = 0.005          # 웨이포인트 사이 최대 간격 (m), 작을수록 더 촘촘하게 직선 보간
MIN_FRACTION = 0.99          # 이보다 덜 완주하면 실패로 처리


def build_cartesian_request(pose: Pose, vel_scale=0.15, acc_scale=0.05):
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = Pose()
    target.position.x, target.position.y, target.position.z = eef_pos.tolist()
    target.orientation = pose.orientation

    req                              = GetCartesianPath.Request()
    req.header.frame_id              = BASE_FRAME
    req.start_state.is_diff          = True
    req.group_name                   = 'lite6'
    req.link_name                    = EEF_LINK
    req.waypoints                    = [target]
    req.max_step                     = MAX_STEP
    req.jump_threshold               = 0.0
    req.avoid_collisions             = True
    req.max_velocity_scaling_factor  = vel_scale
    req.max_acceleration_scaling_factor = acc_scale
    return req


class MoveitCartesianNode(Node):
    def __init__(self):
        super().__init__('moveit2_cartesian')
        self.sub      = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.done_pub = self.create_publisher(Bool, '/ik_done', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self._client  = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._busy    = False
        self.get_logger().info('moveit2_cartesian ready — waiting for /target_pose')

    def _cb_target_pose(self, msg: Pose):
        if self._busy:
            self.get_logger().warn('Already moving, ignoring')
            return

        self._busy = True
        self._send_request(msg)

    def _send_request(self, pose: Pose):
        if not self._client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('compute_cartesian_path service not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        req = build_cartesian_request(pose)
        self._client.call_async(req).add_done_callback(self._response_cb)

    def _response_cb(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error('[FAIL] compute_cartesian_path call failed')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        fraction = result.fraction
        if fraction < MIN_FRACTION:
            self.get_logger().error(
                f'[FAIL] Cartesian path only {fraction:.2%} complete '
                f'(straight line blocked by limit/collision/singularity), aborting')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        traj = result.solution.joint_trajectory
        traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub.publish(traj)
        self.get_logger().info(
            f'[OK] Cartesian trajectory published (fraction={fraction:.2%}, {len(traj.points)} points)')

        if traj.points:
            t = traj.points[-1].time_from_start
            delay = t.sec + t.nanosec * 1e-9 + 0.5
        else:
            delay = 1.0
        self._exec_wait_timer = self.create_timer(delay, self._on_traj_exec_done)

    def _on_traj_exec_done(self):
        self._exec_wait_timer.cancel()
        self._busy = False
        self.done_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MoveitCartesianNode()
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
