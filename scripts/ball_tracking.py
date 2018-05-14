#!/usr/bin/env python
# import the necessary packages


from sensor_msgs.msg import JointState
from std_msgs.msg import String
import rospy
from cv_bridge import CvBridge, CvBridgeError
import roslib
import numpy as np
import argparse
import imutils
import cv2
from collections import deque
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import json


class Ball_Tracker:
    def __init__(self):
        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        self.greenLower = (1, 66, 84)  # 29 86 6
        self.greenUpper = (16, 255, 255)  # 64 255 255
        self.pts = deque(maxlen=100)
        self.bridge = CvBridge()

        try:
            # Initialize node
            rospy.init_node('Ball_Tracker')
            rospy.Subscriber("/camera1/image_raw", Image, self.image_callback)
            self.pub = rospy.Publisher(
                '/bogey1/ball_tracking', Point, queue_size=10)
            self.response_pub = rospy.Publisher('sensors',
                                                String, queue_size=10)

        except:
            print("Failed to Start")
            exit()

        self.W_Res = 800
        self.state = "off"
        self.orange_ball = False

    def image_callback(self, data):
        self.W_Res = 1000
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=self.W_Res)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        # outPUT=cv2.bitwise_and(frame,frame,mask = mask)
        # cv2.imshow("Frame", outPUT)
        # key = cv2.waitKey(1) & 0xFF
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            if not self.orange_ball:
                self.orange_ball = True
                self.response_pub.publish(json.dumps({
                    "sensor": "ball_tracking",
                    "orange_ball": self.orange_ball
                }))
                print "Sent True"
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            point = Point()
            point.x = x
            point.y = y
            point.z = radius
            self.pub.publish(point)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # update the points queue
                self.pts.appendleft(center)
                for i in xrange(1, len(self.pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if self.pts[i - 1] is None or self.pts[i] is None:
                        continue

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(100 / float(i + 1)) * 2.5)
                    # cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        elif self.orange_ball:
            self.orange_ball = False
            self.response_pub.publish(json.dumps({
                "sensor": "ball_tracking",
                "orange_ball": self.orange_ball
            }))
            print "Sent False"

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

    def run(self):
        # keep looping
        while not rospy.is_shutdown():
            rospy.spin()
            # rate.sleep()

        # loop over the set of tracked points

        # cleanup the camera and close any open windows
        self.camera.release()
        cv2.destroyAllWindows()


tracker = Ball_Tracker()
tracker.run()
