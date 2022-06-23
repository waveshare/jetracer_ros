#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","cv_video")

    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10);

  def callback(self,data):
    try:
      img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Our operations on the frame come here
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    cv.imshow("OpenCV Video window", gray)
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

