import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

# Camera -> robot_base extrinsic: p_base = T_cam_base @ p_cam (m). Hand-eye calibration
# ~/Projects/xarm_lite6/cam2robot_pos_calibration/result/T_base_cam.yaml (touch/board_model, 2026-10-27 16:45, RMS 1.1 mm)
T_cam_base = np.array([
    [-0.00169203, -0.99945118,  0.03308287,  0.25562417],
    [-0.97936149,  0.00834258,  0.20194425, -0.25696641],
    [-0.20210942, -0.03205840, -0.97883811,  0.71477754],
    [ 0.0,         0.0,         0.0,         1.0],
])
R_cam_base = T_cam_base[:3, :3]
t_cam_base = T_cam_base[:3, 3]


def cam_to_base(p_cam_xyz):
    p_cam_h = np.append(np.asarray(p_cam_xyz, dtype=float), 1.0)
    p_base_h = T_cam_base @ p_cam_h

    return p_base_h[:3]


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_cam_to_base_publisher')
        self.pub = self.create_publisher(Pose, '/target_pose', 10)

    def publish_base_xyz(self, xyz, rpy_deg=(180, 0, 0)):
        q = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_quat()
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = xyz.tolist()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        self.pub.publish(msg)
        self.get_logger().info(f'published base xyz={np.round(xyz,4)} rpy_deg={rpy_deg}')


def main():
    p_cam_test = np.array([-0.1, -0.2, 0.68]) # target pose

    p_base = cam_to_base(p_cam_test)
    print(f'p_cam = {p_cam_test}  ->  p_base = {np.round(p_base, 4)}')

    rclpy.init()
    node = TestPublisher()

    for _ in range(20):
        if node.pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.get_logger().warn('No subscriber found on /target_pose after 2s — publishing anyway.')

    node.publish_base_xyz(p_base)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
