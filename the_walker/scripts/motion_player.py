#!/usr/bin/env python

import rospy
import time
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import os

global in_action
in_action = 0

controllers = [
    'fl_hip_rotate_controller',
    'fl_hip_left_right_controller',
    'fl_hip_front_back_controller',
    'fl_knee_front_back_controller',
    'fl_foot_front_back_controller',
    'fl_foot_left_right_controller',
    'bl_hip_rotate_controller',
    'bl_hip_left_right_controller',
    'bl_hip_front_back_controller',
    'bl_knee_front_back_controller',
    'bl_foot_front_back_controller',
    'bl_foot_left_right_controller',
    'fr_hip_rotate_controller',
    'fr_hip_left_right_controller',
    'fr_hip_front_back_controller',
    'fr_knee_front_back_controller',
    'fr_foot_front_back_controller',
    'fr_foot_left_right_controller',
    'br_hip_rotate_controller',
    'br_hip_left_right_controller',
    'br_hip_front_back_controller',
    'br_knee_front_back_controller',
    'br_foot_front_back_controller',
    'br_foot_left_right_controller'
]

publishers = []

for controller in controllers:
    publishers.append(rospy.Publisher('/'+controller+'/command', Float64, queue_size=10))


def get_joint_positions(line):
    joint_positions = line.split(',')
    delay = joint_positions[len(publishers)]
    del joint_positions[len(publishers)]

    return (joint_positions, delay)


def motion_play(data):
    global in_action
    if in_action == 0:
        in_action = 1
                
        homedir = os.environ['HOME']
        filepath = homedir + "/catkin_ws/src/the_walker/motions/" + data.data + ".txt"  
                
        try:
            with open(filepath) as f:
                content = f.readlines()
                # you may also want to remove whitespace characters like `\n` at the end of each line
                content = [x.strip('\n') for x in content]

                print "playing " + data.data + " motion"
                for i in range (0, len(content)):
                    (joint_positions, delay) = get_joint_positions(content[i])

                    for i, joint_position in enumerate(joint_positions):
                        publishers[i].publish(float(joint_position))

                        #delay between motions
                        rospy.sleep(float(delay))
                in_action = 0
                print "motion is done"
        except:
            in_action = 0
            print "motion file doens't exist"
    else:
        print "The robot can't play this motion because it is playing another motion. Wait until the motion is done."
        

# call back function for the keyboard/key topic subscribe
def keyboard_capture(data):
        global button
        button = data.code


def motion_control():
        rospy.init_node('motion_player', anonymous=True)
        rospy.Subscriber('/play_motion', String, motion_play)
        rate = rospy.Rate(20)
        rospy.spin()


if __name__ == '__main__':
        try:
                motion_control()
        except rospy.ROSInterruptException:
                pass
