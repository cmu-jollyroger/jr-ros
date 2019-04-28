#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from jr_communication.srv import MotorCmd

class ControlModes():
    CHASSIS_RELAX   = 0 # /**< Chassis no power */
    CHASSIS_STOP    = 1 # /**< Chassis is stopped/breaking */
    CHASSIS_MOVING  = 4 # /**< Chassis is moving */

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    # vertical left stick axis = linear rate
    twist.linear.x = 4*data.axes[1]
    # horizontal left stick axis = turn rate
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)

    chassis_cmd_client(data)

def chassis_cmd_client(data):
    rospy.wait_for_service('jr_comm_cmd')
    try:
        jr_comm_proxy = rospy.ServiceProxy('jr_comm_cmd', MotorCmd)
        #cmd = MotorCmd()
        ctrl_mode = ControlModes.CHASSIS_MOVING
        x_spd = 400 * data.axes[1]
        y_spd = 0# 100 * data.axes[0]
        x_offset = 0
        y_offset = 0
        w_spd = 50 * data.axes[0]
        reset = 0

        # Translation
        x_spd += 400 * data.axes[5]
        y_spd += 400 * data.axes[4]
        #cmd = MotorCmd(ctrl_mode, x_spd, y_spd, x_offset, y_offset, w_spd)
        p_cmd = [ctrl_mode, x_spd, y_spd, x_offset, y_offset, w_spd, reset]
        print p_cmd
        resp = jr_comm_proxy(ctrl_mode, x_spd, y_spd, x_offset, y_offset, w_spd, reset)
        return resp
    except rospy.ServiceException, e:
        print "chassis service call failed: %s"%e

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback, queue_size=1)
    # starts the node
    rospy.init_node('jr_joy_node')
    rospy.spin()

if __name__ == '__main__':
    start()

