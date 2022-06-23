#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2 as cv
import numpy.matlib 
import numpy as np

import random
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","AR_image")

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    self.image_info_sub = rospy.Subscriber(self.camera_name+"/camera_info",CameraInfo,self.info_callback)
    self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10)

    self.mtx = None
    self.dist = None

    self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    self.objp = np.zeros((5*7,3), np.float32)
    self.objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
    self.axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],[0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

  def info_callback(self,data):
    self.dist = np.array(data.D).reshape(1,5)
    self.mtx = np.array(data.K).reshape(3,3)

  def callback(self,data):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    gray = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (7,5),None,cv.CALIB_CB_FAST_CHECK)
    if ret == True:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
        # Find the rotation and translation vectors.
        ret,rvecs, tvecs = cv.solvePnP(self.objp, corners2, self.mtx, self.dist)
        # project 3D points to image plane
        imgpts, jac = cv.projectPoints(self.axis, rvecs, tvecs, self.mtx, self.dist)

        imgpts = np.int32(imgpts).reshape(-1,2)
        # draw ground floor in green
        cv_image = cv.drawContours(cv_image, [imgpts[:4]],-1,(0,255,0),-3)
        # draw pillars in blue color
        for i,j in zip(range(4),range(4,8)):
            cv_image = cv.line(cv_image, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
        # draw top layer in red color
        cv_image = cv.drawContours(cv_image, [imgpts[4:]],-1,(0,0,255),3)

    cv.imshow('3D Image window',cv_image)
    cv.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
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

