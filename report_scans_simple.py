#! /usr/bin/env python
#report_scans_simple
#logs LaserScan data from turtlebot

import rospy
from sensor_msgs.msg import LaserScan
import math

def callback(msg):
    print('=================================')
    left_samples = msg.ranges[90:1:-1]    #goes all the way to ALMOST the center, so if the front distance is the smallest it'll report that
    center = msg.ranges[0]
    right_samples = msg.ranges[-1:-90:-1] #goes all the way to ALMOST the center, so if the front distance is the smallest it'll report that

    #print('{}'.format(left_samples))
    rospy.loginfo('|| {:.4f} | {:.4f} | {:.4f} ||'.format(min(left_samples), center, min(right_samples)))


if __name__ == '__main__':
    try: 

        print("report_scan_simple is running!")
        #creates a node of the name filter_scan
        rospy.init_node('report_scan_simple')
        print("||Min Left | Min | Min Right||")

        #run report
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

