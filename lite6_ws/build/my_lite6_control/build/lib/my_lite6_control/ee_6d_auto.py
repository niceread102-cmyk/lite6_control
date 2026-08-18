import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

DEFAULT_RPY = (180.0, 0.0, 0.0)

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.target_pub = self.create_publisher(Pose, '/target_pose', 10)

    def publish_target(self, x, y, z, roll=None, pitch=None, yaw=None):
        if roll is None:
            roll, pitch, yaw = DEFAULT_RPY

        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True) # extrinsic
        r = Rotation.from_euler('XYZ', [roll, pitch, yaw], degrees=True) # intrinsic        
        q = r.as_quat()  # [x, y, z, w]
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        self.target_pub.publish(msg)
        self.get_logger().info(
            f'Published: pos=[{x:.3f}, {y:.3f}, {z:.3f}] '
            f'rpy=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}] '
            f'quat=[{q[3]:.6f}, {q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}]')

    def publish_target_quat(self, x, y, z, qx, qy, qz, qw):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.target_pub.publish(msg)
        self.get_logger().info(
            f'Published: pos=[{x:.3f}, {y:.3f}, {z:.3f}] '
            f'quat=[{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    try:
        while rclpy.ok():
            user_input = input("input target pos x y z [roll pitch yaw | qx qy qz qw] (or type 'q' to quit): ")

            if user_input.lower() == 'q':
                break

            values = user_input.split()
            try:
                if len(values) == 3:
                    x, y, z = map(float, values)
                    node.publish_target(x, y, z)
                elif len(values) == 6:
                    x, y, z, roll, pitch, yaw = map(float, values)
                    node.publish_target(x, y, z, roll, pitch, yaw)
                elif len(values) == 7:
                    x, y, z, qx, qy, qz, qw = map(float, values)
                    node.publish_target_quat(x, y, z, qx, qy, qz, qw)
                else:
                    raise ValueError

            except ValueError:
                print("Invalid input. Enter 3 numbers (x y z), 6 numbers (x y z roll pitch yaw), or 7 numbers (x y z qx qy qz qw).")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()