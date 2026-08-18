import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
import numpy as np

EE_INIT_R  = np.array([[1,  0,  0],
                       [0, -1,  0],
                       [0,  0, -1]])
                       
M          = np.array([[0,  1,  0],
                       [0,  0,  1],
                       [1,  0,  0]])
BASE_FRAME = 'link_base'
EEF_FRAME  = 'link_eef'


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_new_publisher')
        self.target_pub = self.create_publisher(Pose, '/target_pose', 10)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def print_current_ee(self, timeout_sec=3.0):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec))
        except Exception as e:
            self.get_logger().error(f'TF lookup {BASE_FRAME}->{EEF_FRAME} failed: {e}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        xyz = np.array([t.x, t.y, t.z])
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        self.get_logger().info(
            f'current EE ({EEF_FRAME}) xyz=[{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}] '
            f'rpy(deg)=[{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]')

    def publish_target(self, x, y, z, roll, pitch, yaw):
        R_obj  = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
        R_final = M @ R_obj

        tcp_pos = np.array([x, y, z])

        q = Rotation.from_matrix(R_final).as_quat()  # [x, y, z, w]

        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = tcp_pos.tolist()
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        self.target_pub.publish(msg)
        self.get_logger().info(
            f'tcp=[{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}] '
            f'quat=[{q[3]:.3f}, {q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}]')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.print_current_ee()

    try:
        while rclpy.ok():
            user_input = input("input tcp pos x y z roll pitch yaw (or type 'q' to quit): ")

            if user_input.lower() == 'q':
                break

            try:
                x, y, z, roll, pitch, yaw = map(float, user_input.split())
                node.publish_target(x, y, z, roll, pitch, yaw)

            except ValueError:
                print("Invalid input. Please enter exactly 6 numbers separated by space.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
