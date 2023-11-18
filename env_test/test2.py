#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

            # Nhận diện đường tròn
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=50)

            # Vẽ các đường tròn lên ảnh
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # Vẽ đường tròn ngoại vi
                    cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Vẽ tâm đường tròn
                    cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

            # Hiển thị ảnh
            cv2.imshow("RealSense Image with Circles", cv_image)
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
