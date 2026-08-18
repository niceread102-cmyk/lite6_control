import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from scipy.spatial.transform import Rotation

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.target_pub = self.create_publisher(Pose, '/target_pose', 10)

    def publish_target(self, x, y, z, roll, pitch, yaw):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        
        r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        q = r.as_quat()  # [x, y, z, w]
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        self.target_pub.publish(msg)
        self.get_logger().info(f'Published: pos=[{x:.2f}, {y:.2f}, {z:.2f}] quat=[{q[3]:.3f}, {q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}]')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    try:
        while rclpy.ok():
            user_input = input("input target pos x y z roll pitch yaw (or type 'q' to quit): ")
            
            if user_input.lower() == 'q':
                break
                
            try:
                target_x, target_y, target_z, target_roll, target_pitch, target_yaw = map(float, user_input.split())
                node.publish_target(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
                
            except ValueError:
                print("Invalid input. Please enter exactly 6 numbers separated by space.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()