#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class RealSenseBlackThresholding:
    def __init__(self):
        rospy.init_node('realsense_black_thresholding', anonymous=True)
        self.bridge = CvBridge()

        # Đăng ký subscriber cho ảnh màu từ camera RealSense
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Chuyển đổi message ROS Image sang ảnh OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Chuyển đổi ảnh từ BGR sang mức xám
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Xác định ngưỡng màu đen
            _,black_threshold = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)
            #
            qcd = cv2.QRCodeDetector()
            retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(gray_image)
            if retval:
                print(f'Decoded information: {decoded_info}')
                #print(points)
                #or point in points:
                    #cv2.polylines(black_threshold, [point], isClosed=True, color=(0, 255, 0), thickness=2)
                gray_image = cv2.polylines(gray_image, points.astype(int), True, (0, 255, 0), 3)
                #cv2.imshow('Binary Image with QR Code', img)
                #cv2.waitKey(0)
                #cv2.destroyAllWindows()
            else:
                print('No QR code detected.')
            #
            cv2.imshow('Original Image', cv_image)
            cv2.imshow('Black Thresholding Result', gray_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("Error converting Image message: %s" % str(e))

if __name__ == '__main__':
    try:
        black_thresholding = RealSenseBlackThresholding()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        cv2.destroyAllWindows()
