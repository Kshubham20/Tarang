#! /usr/bin/env python

import rospy
# from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
rospy.init_node('perform_gate_task')

model_state_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

command = ModelState()

command.model_name = 'triton'
vel = 0.5
r = rospy.Rate(50)
curtime = rospy.Time.now()
lastime = rospy.Time.now()
x = 0
z = -3
while not rospy.is_shutdown():
	curtime = rospy.Time.now()
	diff = (lastime - curtime).to_sec()
	x = x+ diff*vel
	command.pose.position.x = x
	command.pose.position.z = z
	model_state_publisher.publish(command)
	lastime = curtime
	r.sleep()