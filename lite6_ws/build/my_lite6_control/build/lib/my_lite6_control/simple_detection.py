import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


# gear color
HSV_LOW  = np.array([ 85, 30, 150])
HSV_HIGH = np.array([130, 255, 255])

MIN_AREA = 300


class SimpleDetection(Node):
    def __init__(self):
        super().__init__('simple_detection')

        self.bridge           = CvBridge()
        self.cam_sub          = self.create_subscription(Image, '/wrist_camera/image_raw', self._cb_image, 10)
        self.gear_pub         = self.create_publisher(Point, '/gear_center', 10)

        self.cx               = None
        self.cy               = None
        self.get_logger().info("SimpleDetection ready. Detecting sky-blue gear.")

    def _cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cy, self.cx = int(frame.shape[0] / 2), int(frame.shape[1] / 2)
        self.get_logger().info("%d, %d" %(self.cx, self.cy))
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask  = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)

        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) >= MIN_AREA:

                M  = cv2.moments(largest)
                obj_x = M['m10'] / M['m00']
                obj_y = M['m01'] / M['m00']

                loss_x = self.cx - obj_x
                loss_y = self.cy - obj_y
                self.gear_pub.publish(Point(x=obj_x, y=obj_y, z=0.0))
                self.get_logger().info(f"gear center: ({obj_x:.1f}, {obj_y:.1f})")
                
                
                cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                cv2.circle(frame, (int(obj_x), int(obj_y)), 5, (0, 0, 255), -1)

        cv2.imshow("Gear Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
