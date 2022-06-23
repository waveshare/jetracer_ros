#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2 as cv
import datetime
import imutils
import time
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","motion_detect")

    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10);

    self.avg = None

  def callback(self,data):
    try:
      frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert the frame to gray, which can increase the efficiency of analysis.
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Gaussian blur the frame to avoid misjudgment caused by noise.
    gray = cv.GaussianBlur(gray, (21, 21), 0)

    if self.avg is None:
        self.avg = gray.copy().astype("float")

    # background update.
    cv.accumulateWeighted(gray, self.avg, 0.3)

    # Compare the difference between the new frame and the background.
    frameDelta = cv.absdiff(gray, cv.convertScaleAbs(self.avg))
    thresh = cv.threshold(frameDelta, 25, 255, cv.THRESH_BINARY)[1]

    thresh = cv.dilate(thresh, None, iterations=2)
    contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    text = "Unoccupied"
    for c in contours:
        # if the contour is too small, ignore it
        if cv.contourArea(c) < 1500:
            continue

        (x, y, w, h) = cv.boundingRect(c)
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = "Occupied"

    # draw the text and timestamp on the frame
    if(text == "Occupied"):
    	cv.putText(frame, "Status: {}".format(text), (10, 20),
        	cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
        (10, frame.shape[0] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    cv.imshow("Frame Image", frame)
    #cv.imshow("Thresh", thresh)
    cv.imshow("Frame Delta", frameDelta)
    cv.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(gray))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

