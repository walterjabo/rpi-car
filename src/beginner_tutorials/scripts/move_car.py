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

class Car(metaclass=Singleton):

	inA1 = 24
	inA2 = 23
	enA = 25

	inB1 = 22
	inB2 = 27
	enB = 17
	
	def __init__(self) :

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.inA1, GPIO.OUT)
		GPIO.setup(self.inA2, GPIO.OUT)
		GPIO.setup(self.enA, GPIO.OUT)

		GPIO.setup(self.inB1, GPIO.OUT)
		GPIO.setup(self.inB2, GPIO.OUT)
		GPIO.setup(self.enB, GPIO.OUT)

		GPIO.output(self.inA1, GPIO.LOW)
		GPIO.output(self.inA2, GPIO.LOW)
		GPIO.output(self.inB1, GPIO.LOW)
		GPIO.output(self.inB2, GPIO.LOW)

		self.p=GPIO.PWM(self.enA, 1000)
		self.q=GPIO.PWM(self.enB, 1000)


		self.p.start(50)
		self.q.start(50)

		rospy.loginfo(rospy.get_caller_id()+': MOTORES CAR INICIADOS')
	
	def move_wheel_A(self, direction):
		rospy.loginfo('move_wheel_A %s' % direction)
		if direction > 0:
			GPIO.output(self.inA1, GPIO.LOW)
			GPIO.output(self.inA2, GPIO.HIGH)
		else:
			GPIO.output(self.inA1, GPIO.HIGH)
			GPIO.output(self.inA2, GPIO.LOW)

	def move_wheel_B(self, direction):
		rospy.loginfo('move_wheel_B %s' % direction)
		if direction > 0:
			GPIO.output(self.inB1, GPIO.HIGH)
			GPIO.output(self.inB2, GPIO.LOW)
		else:
			GPIO.output(self.inB1, GPIO.LOW)
			GPIO.output(self.inB2, GPIO.HIGH)




def callback(direction):

	car = Car()
	if direction.data == 'car_forward':
		car.move_wheel_A(1)
		car.move_wheel_B(1)
	elif direction.data == 'car_backward':
		car.move_wheel_A(-1)
		car.move_wheel_B(-1)
	elif direction.data == 'car_left':
		car.move_wheel_A(1)
		car.move_wheel_B(-1)
	elif direction.data == 'car_right':
		car.move_wheel_A(-1)
		car.move_wheel_B(1)
	elif direction.data == 'exit':
		GPIO.cleanup()
		car.p.stop
		car.q.stop
		sys.exit('terminado por teclado command exit')
	
def listenerCarMovements():
	rospy.init_node('move_car_wheels', anonymous=False)
	rospy.Subscriber('move_car', String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	try:
		listenerCarMovements()
	except rospy.ROSInterruptException:
		GPIO.cleanup()

