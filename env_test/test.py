import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
class RealSenseImageSubscriber:
    def __init__(self):
        rospy.init_node('realsense_image_subscriber', anonymous=True)
        self.bridge = CvBridge()

        # Đăng ký subscriber cho ảnh màu từ camera RealSense
        self.image_sub = rospy.Subscriber('TB23_916b/camera/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Chuyển đổi message ROS Image sang ảnh OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Hiển thị ảnh
            cv2.imshow("RealSense Image", cv_image)
            cv2.waitKey(1)  # Dừng lại 1 miligiay để hiển thị ảnh

        except CvBridgeError as e:
            rospy.logerr("Error converting Image message: %s" % str(e))

if __name__ == '__main__':
    try:
        realsense_image_subscriber = RealSenseImageSubscriber()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        cv2.destroyAllWindows()
