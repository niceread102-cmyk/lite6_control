import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from my_lite6_control import custom_ik_rotation


np.set_printoptions(precision=3, suppress=True)

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class IK_Node(Node):
    def __init__(self):
        super().__init__('IK_Node')

        self.sub_js = self.create_subscription(JointState, '/joint_states', self._cb_joint_states, 10)
        self.sub    = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.pub    = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.theta  = np.array([1.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 1.0])

    def _cb_joint_states(self, msg: JointState):
        try:
            positions = [msg.position[msg.name.index(n)] for n in JOINT_NAMES]
            self.theta = np.array(positions)
        except ValueError:
            pass

    def _cb_target_pose(self, msg: Pose):
        theta_init = self.theta.copy()
        target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        target_R   = custom_ik_rotation.quat_to_R(msg.orientation.w, msg.orientation.x,
                                                   msg.orientation.y, msg.orientation.z)

        theta, ok = custom_ik_rotation.solve_ik(target_pos, target_R, theta_init)

        if not ok:
            self.get_logger().warn(f"IK failed for pos={np.round(target_pos,3)}")
            return

        delta    = float(np.max(np.abs(theta - theta_init)))
        move_sec = max(3.0, delta * 8.0)

        pt = JointTrajectoryPoint()
        pt.positions       = theta.tolist()
        pt.time_from_start = Duration(sec=int(move_sec), nanosec=int((move_sec % 1) * 1e9))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.pub.publish(jt)
        self.get_logger().info(f"[OK] pos={np.round(target_pos,3)} theta={np.round(theta,3)}")

def main(args=None):
    rclpy.init(args=args)
    node = IK_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()