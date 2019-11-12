#!/usr/bin/env python3
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time, sys


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        # print('--------------------')
        # print(cls)
        # print(cls._instances)
        # print('--------------------')
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class CameraMotor(metaclass=Singleton):
	def __init__(self, value) :
		self.pin = value
		rospy.loginfo(rospy.get_caller_id()+': MOTOR CREADO CON PUERTO: ' + str(self.pin))

		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.pin, GPIO.OUT)
		self.p = GPIO.PWM(self.pin, 50)
		self.pos = 7.4
		self.p.start(self.pos) # Initialization
		time.sleep(0.03)

def callback(direction):

	motor = CameraMotor(12)
	temp = 0
	command_permitted = False
	if direction.data == 'camera_left':
		temp = motor.pos + 0.2
		command_permitted = True
	elif direction.data == 'camera_right':
		temp = motor.pos - 0.2
		command_permitted = True
	elif direction.data == 'exit':
		motor = CameraMotor(12)
		motor.p.stop
		GPIO.cleanup()
		sys.exit('terminado por teclado command exit')

	if command_permitted and temp >= 0 and temp <= 14:
		motor.pos = temp
		motor.p.ChangeDutyCycle(motor.pos)
		rospy.loginfo(rospy.get_caller_id() + ': direction=' + direction.data + ' Camera moved to position ' + str(motor.pos))
	
	
def listener():
	rospy.init_node('move_camera_motor', anonymous=False)
	rospy.Subscriber('move_camera', String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		motor = CameraMotor(12)
		motor.p.stop
		GPIO.cleanup()

