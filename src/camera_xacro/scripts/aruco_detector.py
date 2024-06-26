#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
# import numpy as np

class ArucoDetector:
    
    def __init__(self):
        self.bridge = CvBridge() # This changes ros images to CV images

        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters =self.parameters)

        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Display the image with detected markers
        cv2.imshow("Detected ArUco markers", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
