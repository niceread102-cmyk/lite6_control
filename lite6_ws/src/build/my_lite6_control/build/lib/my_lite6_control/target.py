import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.target_pub = self.create_publisher(Point, '/target_pos', 10)

    def publish_target(self, x, y, z):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z
        self.target_pub.publish(msg)
        self.get_logger().info(f'Published target pos: x={x:.2f}, y={y:.2f}, z={z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    try:
        while rclpy.ok():
            user_input = input("input target pos x y z (or type 'q' to quit): ")
            
            if user_input.lower() == 'q':
                break
                
            try:
                target_x, target_y, target_z = map(float, user_input.split())
                node.publish_target(target_x, target_y, target_z)
            except ValueError:
                print("Invalid input. Please enter exactly 3 numbers separated by space.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()