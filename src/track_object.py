#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time


from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# define the lower and upper boundaries of the "green"
# ball in the HSV color space



class image_converter:
  def __init__(self):

    self.image_pub = rospy.Publisher("opencv_camera",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/gimbal/camera/image_raw",Image,self.callback)
    counter = 0
    (dX, dY) = (0, 0)
    direction = ""
  def callback(self,data):
    greenLower = (29, 86, 6) 
    greenUpper = (64, 255, 255)
    pts = deque(maxlen=32)
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    # frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # cv2.imshow("Blurred", blurred) #present blurred filter
    # cv2.imshow("HSV", hsv)  #present inverted colors
    
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask0 = cv2.inRange(hsv, greenLower, greenUpper)
    mask1 = cv2.erode(mask0, None, iterations=2)
    mask = cv2.dilate(mask1, None, iterations=2)
   
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None


    # only proceed if at least one contour was found
    if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      print (center)

      # only proceed if the radius meets a minimum size
      if radius > 10:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(frame, (int(x), int(y)), int(radius),
          (0, 255, 255), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        pts.appendleft(center)
        	# loop over the set of tracked points

    for i in np.arange(1, len(pts)):
      # if either of the tracked points are None, ignore
      # them
      if pts[i - 1] is None or pts[i] is None:
        continue

      # check to see if enough points have been accumulated in
      # the buffer
      if counter >= 10 and i == 1 and pts[-10] is not None:
        # compute the difference between the x and y
        # coordinates and re-initialize the direction
        # text variables
        dX = pts[-10][0] - pts[i][0]
        dY = pts[-10][1] - pts[i][1]
        (dirX, dirY) = ("", "")

        # ensure there is significant movement in the
        # x-direction
        if np.abs(dX) > 20:
          dirX = "East" if np.sign(dX) == 1 else "West"

        # ensure there is significant movement in the
        # y-direction
        if np.abs(dY) > 20:
          dirY = "North" if np.sign(dY) == 1 else "South"

        # handle when both directions are non-empty
        if dirX != "" and dirY != "":
          direction = "{}-{}".format(dirY, dirX)

        # otherwise, only one direction is non-empty
        else:
          direction = dirX if dirX != "" else dirY

      # otherwise, compute the thickness of the line and
      # draw the connecting lines
      thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
      cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # print ("dx: ", dirX, "dy: ", dirY)
    cv2.imshow("Image window", frame)
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)