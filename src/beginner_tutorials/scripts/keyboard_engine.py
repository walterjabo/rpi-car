#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import sys, termios, tty

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def talker():
    pub_camera = rospy.Publisher('move_camera', String, queue_size=10)
    pub_car = rospy.Publisher('move_car', String, queue_size=10)
    rospy.init_node('keyboard_engine', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x = getch()

        if x=='[':
            rospy.loginfo('tecla cursora: --%s--' % x)
            continue

        rospy.loginfo('You have press tecla: --%s--' % x)

        command = ''
        kind  = ''

        if x == 'a':
            command = 'camera_right'
            kind = 'camera'
        elif x == 'd':
            command = 'camera_left'
            kind = 'camera'
        elif x == 'A':
            command = 'car_forward'
            kind = 'car'
        elif x == 'B':
            command = 'car_backward'
            kind = 'car'
        elif x == 'D':
            command = 'car_left'
            kind = 'car'
        elif x == 'C':
            command = 'car_right'
            kind = 'car'
        elif x == 'e':
            command = 'exit'

        rospy.loginfo('You have press %s' % command)
        if kind == 'camera':
            pub_camera.publish(command)
        elif kind == 'car':
            pub_car.publish(command)
        else:
            pub_camera.publish(command)
            pub_car.publish(command)

        rate.sleep()
        if command == 'exit':
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
