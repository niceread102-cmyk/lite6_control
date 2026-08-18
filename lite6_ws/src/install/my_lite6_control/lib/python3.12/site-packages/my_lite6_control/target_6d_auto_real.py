import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Log
from scipy.spatial.transform import Rotation
import numpy as np

EE_INIT_R     = np.array([[1,  0, 0],
                           [0, -1, 0],
                           [0,  0, -1]])

READY_MSG = '[MOVE] waiting for target pose'

GEARS = [
    ('Gear_1', 0.2,  0.1, 0.05, 0, -20, 180),
    ('Gear_2', 0.2,  0.0, 0.05, 0, -20,   0),
    ('Gear_3', 0.2, -0.1, 0.05, 0, -20, -90),
    ('Gear_4', 0.3,  0.0, 0.05, 0, -20, 180),
    ('Gear_5', 0.3,  0.1, 0.05, 0, -20, 45),
    ('Gear_6', 0.3, -0.1, 0.05, 0, -20, -45),
]


class TargetAutoPublisher(Node):
    def __init__(self):
        super().__init__('target_auto_publisher_real')
        self.target_pub = self.create_publisher(Pose, '/target_pose', 10)
        self.rosout_sub = self.create_subscription(Log, '/rosout', self._cb_rosout, 10)
        self._idx = 0

    def _cb_rosout(self, msg: Log):
        if msg.name != 'sequence_real_node' or msg.msg != READY_MSG:
            return
        if self._idx >= len(GEARS):
            return

        name, x, y, z, roll, pitch, yaw = GEARS[self._idx]
        self._idx += 1
        self.publish_target(name, x, y, z, roll, pitch, yaw)

    def publish_target(self, name, x, y, z, roll, pitch, yaw):
        R_obj   = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
        R_final = R_obj @ EE_INIT_R
        q = Rotation.from_matrix(R_final).as_quat()  # [x, y, z, w]

        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        self.target_pub.publish(msg)
        self.get_logger().info(f'[{name}] published: pos=[{x:.3f}, {y:.3f}, {z:.3f}] quat=[{q[3]:.3f}, {q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}]')
        

def main(args=None):
    rclpy.init(args=args)
    node = TargetAutoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
