#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Setup converter between ROS and opencv images
bridge = CvBridge()

# setup publisher and subscriber
image_pub = rospy.Publisher("/camera/color/image_filtered", Image,
                            queue_size=1)


def callback(data):
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hueLow = 0
    hueUp = 63
    Ls = 7
    Us = 255
    Lv = 65
    Uv = 255

    l_b = np.array([hueLow, Ls, Lv])
    u_b = np.array([hueUp, Us, Uv])

    FGmask = cv2.inRange(hsv, l_b, u_b)

    contours, _ = cv2.findContours(FGmask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        (x, y, w, h) = cv2.boundingRect(cnt)
        if area >= 50:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)

    try:
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_msg.header = data.header
        image_pub.publish(image_msg)
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    # Setup the node
    rospy.init_node('image_filter', anonymous=True)
    # Setup the subscriber
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback,
                                 queue_size=1)
    # Run untill shutdown
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
