#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def take_action(regions):
    msg = Twist()
    LIDAR_MIN = 0.7


    if min(regions[1:3]) > LIDAR_MIN:
        state_description = 'No_obstacle'
        msg.linear.x = 1.0

    else:
        state_description = 'Obstacle_detected'
        msg.linear.x = 0.0
    
    rospy.loginfo(state_description)
    pub.publish(msg)

def lidar_read(msg):
    regions = [
        round(min(msg.ranges[0:143]),2),
        round(min(msg.ranges[144:287]),2),
        round(min(msg.ranges[288:431]),2),
        round(min(msg.ranges[432:575]),2),
        round(min(msg.ranges[576:713]),2),
    ]
    take_action(regions)

def main():
    global pub

    rospy.init_node('obstacledetection')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/scout/laser/scan", LaserScan, lidar_read)
    rospy.spin()

if __name__ == '__main__':
    main()