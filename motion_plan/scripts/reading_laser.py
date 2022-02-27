#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan

def lidar_read(msg):

    detection = [
        round(min(msg.ranges[0:143]),2),
        round(min(msg.ranges[144:287]),2),
        round(min(msg.ranges[288:431]),2),
        round(min(msg.ranges[432:575]),2),
        round(min(msg.ranges[576:713]),2),
    ]
    rospy.loginfo(detection)

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/scout/laser/scan", LaserScan, lidar_read)

    rospy.spin()

if __name__ == '__main__':
    main()
