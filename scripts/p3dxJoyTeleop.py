#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop(object):
	'''
	A node that listens to joystick output, scales the values, and publishes
	them as velocity commands.
	'''

	def __init__(self, linearAxisIndex = 3, angularAxisIndex = 2):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		rospy.init_node('p3dxJoyTeleop')

		self.lin_spd = 0
		self._LinearAxisIndex = rospy.get_param("~linearAxisIndex", linearAxisIndex)
		self._AngularAxisIndex = rospy.get_param("~angularAxisIndex", angularAxisIndex)
		self._LinearScalingFactor = rospy.get_param("~linearScalingFactor", 0.2)
		self._AngularScalingFactor = rospy.get_param("~angularScalingFactor", 0.5)

		rospy.loginfo("Starting teleop node with linear axis %d and angular axis %d" % (self._LinearAxisIndex, self._AngularAxisIndex))

		# subscriptions
		rospy.Subscriber("/joy", Joy, self._HandleJoystickMessage)
		self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

	def _HandleJoystickMessage(self, joyMessage):
		rospy.logwarn("Handling joystick message: " + str(joyMessage))
		ang = 0
		MAX_FORWARD = 0.5
		MIN_FORWARD = -0.5

		#controle do lab 3 1 9 0 2 //controle nosso 0 2 9 3 1
		# triangle
		if (joyMessage.buttons[0] == 1 and (self.lin_spd <= MAX_FORWARD)):
			self.lin_spd += 0.05

		# x
		if (joyMessage.buttons[2] == 1 and (self.lin_spd >= MIN_FORWARD)):
			self.lin_spd -= 0.05

		# start
		if (joyMessage.buttons[9] == 1):
			self.lin_spd = 0

		# square
		if (joyMessage.buttons[3] == 1):
			ang = 0.5

		# ball
		if (joyMessage.buttons[1] == 1):
			ang = -0.5

		velocityCommand = Twist()
		velocityCommand.linear.x = self.lin_spd # self._LinearScalingFactor * joyMessage.axes[1]
		velocityCommand.angular.z = self._AngularScalingFactor * ang
		
		self._VelocityCommandPublisher.publish(velocityCommand)


if __name__ == '__main__':
	teleop = Teleop()
	rospy.spin()

