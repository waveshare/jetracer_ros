#!/usr/bin/env python
import roslib
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from jetracer.cfg import laserFilterConfig

class LaserFilter:
    def __init__(self):
        self.Angle = 360
        self.Dist = 12
        self.sub = rospy.Subscriber("scan",LaserScan,self.callback)
        self.pub = rospy.Publisher("filteredscan",LaserScan,queue_size=10)
        Server(laserFilterConfig, self.config_callback)

    def config_callback(self, config, level):
        self.Angle = config['laserAngle']
        self.Dist = config['distance']
        return config

    def callback(self,data):
        newdata = data
        
        #Convert tuple data to list
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)
        #data len
        length = len(data.ranges)
        #Angle range
        Index = int(self.Angle/2*length/360)

        #Distance filtering
        for i in range(length):
            if(newdata.ranges[i] > self.Dist):
                newdata.ranges[i] = 0
                newdata.intensities[i] = 0
                
        #Angle filtering
        for i in range(Index,length -Index):
            newdata.ranges[i] = 0
            newdata.intensities[i] = 0
            
        self.pub.publish(newdata)

if __name__ == '__main__':

    rospy.init_node('LidarFilter',anonymous=False)
    laser = LaserFilter()

    rospy.spin()
