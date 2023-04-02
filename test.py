import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = np.empty(shape=[0])
color = (0, 255, 0)
class ROSPOSE(Node):
    def __init__(self):
        super().__init__('pose')
        self.subscriber = self.create_subscription(
            Image, '/camera/color/image_raw', self.sub_callback, 10) #/camera/color/image_raw
    def sub_callback(self, data):
        global cv_image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        if cv_image.size == (640*480*3):
            pass
        #cv2.namedWindow('Pose Classification', cv2.WINDOW_NORMAL)
        frame = cv_image

        # Display the frame.
        cv2.imshow('Marshaller Pose Estimate', frame)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
def main(args=None):
    rclpy.init(args=args)
    POSE = ROSPOSE()
    try:
        rclpy.spin(POSE)
    except KeyboardInterrupt:
        POSE.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        POSE.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
  main()