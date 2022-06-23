#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2 as cv
import random
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","contours_image")

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10)

  def callback(self,data):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    edges_img = cv.Canny(cv_image,50,200)
    contours_img = cv.cvtColor(edges_img, cv.COLOR_GRAY2RGB)

    contours, hierarchy = cv.findContours(edges_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
	cv.drawContours(contours_img,contours,i,(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)),1)

    cv.imshow("Image window", cv_image)
    cv.imshow("Edges window", edges_img)
    cv.imshow("Contours window", contours_img)
    cv.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(contours_img))
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

