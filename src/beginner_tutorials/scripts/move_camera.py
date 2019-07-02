#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time, sys


class CameraMotor:
	class __CameraMotor:
		def __init__(self, arg):
			self.pin = arg

			rospy.loginfo(rospy.get_caller_id()+': MOTOR CREADO CON PUERTO: ' + str(self.pin))


			GPIO.setmode(GPIO.BOARD)
			GPIO.setup(self.pin, GPIO.OUT)
			self.p = GPIO.PWM(self.pin, 50)
			self.pos = 7.4
			self.p.start(self.pos) # Initialization
			time.sleep(0.03)


		def __str__(self):
			return repr(self) + self.pin
	instance = None
	def __init__(self, arg):
		if not CameraMotor.instance:
			CameraMotor.instance = CameraMotor.__CameraMotor(arg)
		else:
			CameraMotor.instance.pin = arg
	def __getattr__(self, name):
		return getattr(self.instance, name)

	

def callback(direction):

	motor = CameraMotor(12)
	temp = 0

	rospy.loginfo('direction: ' + direction.data + ' motor.pos: ' + str(motor.pos))

	if direction.data == 'left':
		temp = motor.pos + 0.2
	elif direction.data == 'right':
		temp = motor.pos - 0.2
	else:
		motor = CameraMotor(12)
		motor.p.stop
		GPIO.cleanup()
		sys.exit('terminado por teclado command exit')

	rospy.loginfo('temp value: ' + str(temp))

	if temp >= 0 and temp <= 14:
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

