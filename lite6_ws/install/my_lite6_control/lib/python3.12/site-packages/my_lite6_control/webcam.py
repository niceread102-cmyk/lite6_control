import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from my_lite6_control.aruco import Aruco

CAM_TOPIC = '/wrist_camera/image_raw'

class WebCam(Node):
    def __init__(self):
        super().__init__('webcam_node')

        self.cam_sub            = self.create_subscription(Image, CAM_TOPIC, self._cb_image, 10)
        self.target_sub         = self.create_subscription(Int32, '/aruco/target_id', self._cb_target_id, 10)
        self.aruco_pos_pub      = self.create_publisher(Point, '/aruco_pos',        10)
        self.aruco_center_pub   = self.create_publisher(Point, '/aruco_center',     10)
        self.frame_center_pub   = self.create_publisher(Point, '/frame_center',     10)
        self.detected_id_pub    = self.create_publisher(Int32, '/aruco/detected_id', 10)

        self.bridge    = CvBridge()
        self.aruco     = Aruco()
        self.target_id = 0

        self.get_logger().info(f"WebCam node started. topic: {CAM_TOPIC}")

    def _cb_target_id(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"target_id → {self.target_id}")

    def _cb_image(self, msg):
        try:
            frame    = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            fy, fx   = frame.shape[:2]
            corners, ids = self.aruco.detect(frame)

            if ids is not None:
                for detected_id in ids.flatten():
                    self.detected_id_pub.publish(Int32(data=int(detected_id)))
                result = self.aruco.aruco_marker_center(corners, ids, frame, fy, fx, self.target_id)
                if result is not None:
                    mx, my, cx, cy = result
                    self.aruco_center_pub.publish(Point(x=float(mx), y=float(my), z=0.0))
                    self.frame_center_pub.publish(Point(x=float(cx), y=float(cy), z=0.0))

                    # print(f"aruco: ({mx:.1f}, {my:.1f})  frame: ({cx:.1f}, {cy:.1f})")

            cv2.imshow("Wrist Camera", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebCam()

    def _shutdown(sig, frame):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()