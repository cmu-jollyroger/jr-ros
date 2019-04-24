#!/usr/bin/env python
import rospy

# Intializes everything
def start():
    # starts the node
    rospy.init_node('jr_device_node')
    rospy.spin()

if __name__ == '__main__':
    start()

