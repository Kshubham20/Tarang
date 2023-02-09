#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos, pi

import sys
import select
import termios
import tty

msg = """
Control Triton!
---------------------------
Moving around:
   w : Move forward
   s : Move backwards
   a : Yaw left 
   d : Yaw right
   q : heave down
   e : heave up
u/j : increase/decrease foward speed by 10%
i/k : increase/decrease only yaw speed by 10%
o/l : increase/decrease only heave speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0), # Move forward
    's': (-1, 0, 0), # Move backwards
    'd': (0,-1, 0), # Yaw turn left 
    'a': (0, 1, 0), # Yaw turn Right
    'q': (0, 0, 1), # Heave Up
    'e': (0, 0, -1) # Heave Down
}

speedBindings = {
    'u': (1.1, 1, 1),
    'i': (1, 1.1, 1),
    'o': (1, 1, 1.1),
    'j': (0.9, 1, 1),
    'k': (1, 0.9, 1),
    'l': (1, 1, 0.9)
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1
heave_speed = .2

def vels(speed, turn, heave_speed):
    return "currently:\tspeed %s\tturn %s \theave_speed %s " % (speed, turn, heave_speed)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)

    x = 0
    th = 0
    depth = 0
    
    status = 0
    count = 0
    acc = 0.1
    
    target_speed = 0
    target_turn = 0
    target_heave_speed = 0

    control_speed = 0
    control_turn = 0
    control_heave_speed = 0
    
    try:
        print(msg)
        print(vels(speed, turn, heave_speed))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                depth = moveBindings[key][2]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                heave_speed = heave_speed * speedBindings[key][2]
                count = 0

                print(vels(speed, turn, heave_speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k':
                x = 0
                th = 0
                depth = 0
                control_speed = 0
                control_turn = 0
                control_heave_speed = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th
            target_heave_speed = heave_speed * depth
            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            if target_heave_speed > control_heave_speed:
                control_heave_speed = min(target_heave_speed, control_heave_speed + 0.02)
            elif target_heave_speed < control_heave_speed:
                control_heave_speed = max(target_heave_speed, control_heave_speed - 0.02)
            else:
                control_heave_speed = target_heave_speed
            command = ModelState()
            command.model_name = 'triton'
            command.reference_frame = 'world'
            
            rospy.wait_for_service('/gazebo/get_model_state')
            get_model_srv = rospy.ServiceProxy(
                '/gazebo/get_model_state', GetModelState)
            model = GetModelStateRequest()
            model.model_name = 'triton'
            result = get_model_srv(model)

            orientation_list = [result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            new_yaw = yaw + 0.05*control_turn

            command.pose.position.x = result.pose.position.x + 0.1*control_speed*cos(new_yaw)
            command.pose.position.y = result.pose.position.y + 0.1*control_speed*sin(new_yaw)
            command.pose.position.z = result.pose.position.z + 0.1*control_heave_speed

            new_orientation_list = quaternion_from_euler(roll, pitch, new_yaw)
            command.pose.orientation.x = new_orientation_list[0]
            command.pose.orientation.y = new_orientation_list[1]
            command.pose.orientation.z = new_orientation_list[2]
            command.pose.orientation.w = new_orientation_list[3]

            pub.publish(command)
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print('e')

    finally:
        # Last ending message
        # strmsg = String()
        # strmsg.data = "Shubham Korde 2"
        # pub.publish(strmsg)
        command = ModelState()
        command.model_name = 'triton'
        pub.publish(command)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
