#! /usr/bin/env python
"""Node to control the bot using teleop keyboard"""

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
import sys, time
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

VERBOSE = True
class  teleop_keyboard_node:
	"""Teleop Keyboard class"""
	# Initialising the variables for positon and velocity
	x = 0 
	y = 0
	z = 0
	roll = 0
	pitch = 0
	yaw = 0
	vel_x = 0
	vel_y = 0
	vel_z = 0
	vel_roll = 0
	vel_pitch = 0
	vel_yaw = 0
	# Variable used to calculate the integral
	last_time  = 0.0
	current_time = 0.0
	command = ModelState()
	def __init__(self):
		"""	Initialize the subscriber to subscribe from DVL topic and
			Publisher to publish on the desired topic
		"""
		self.pose_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		# Initialise time
		self.last_time = rospy.Time.now()
		self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=10)
		if VERBOSE:
			print("subscribed to /cmd_vel")

	def callback(self, data):
		"""callback function of the subscribed topic publishing cmd_vel"""
		self.vel_x = data.linear.x
		self.vel_y = data.linear.y
		self.vel_z = data.linear.z
		self.vel_roll = data.angular.x
		self.vel_pitch = data.angular.y
		self.vel_yaw = data.angular.z
		
	def loop_function(self):
		self.current_time = rospy.Time.now()
		dt = (self.current_time - self.last_time).to_sec()
		self.x = self.x + dt*self.vel_x
		self.y = self.y + dt*self.vel_y
		self.z = self.z + dt*self.vel_z
		self.roll = self.roll + dt*self.vel_roll
		self.pitch = self.pitch + dt*self.vel_pitch
		self.yaw = self.yaw + dt*self.vel_yaw

		self.command.model_name = 'triton'
		self.command.pose.position.x = self.x
		self.command.pose.position.y = self.y
		self.command.pose.position.z = self.z
		quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
		# type(pose) = geometry_msgs.msg.Pose
		self.command.pose.orientation.x = quaternion[0]
		self.command.pose.orientation.y = quaternion[1]
		self.command.pose.orientation.z = quaternion[2]
		self.command.pose.orientation.w = quaternion[3]
		# self.command.twist.linear.x = self.vel_x
		# self.command.twist.linear.y = self.vel_y
		# self.command.twist.linear.z = self.vel_z

def main(args):
	"""Intialising and Cleaning Up of ROS Nodes"""
	rospy.init_node('teleop_control')
	ic = teleop_keyboard_node()
	r = rospy.Rate(10) # 10hz 
	while not rospy.is_shutdown():
		ic.pose_publisher.publish(ic.command)
		rospy.loginfo("published the message")
		rospy.loginfo(ic.command)
		ic.loop_function()
		r.sleep()
	try: 
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down the Teleop Keyboard Node")

if __name__ == '__main__':
	main(sys.argv)

