#!/usr/bin/env python

# import libraries, message types, and service types
import rospy
import os
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_msgs.msg import JointState
from keyboard.msg import Key
from std_msgs.msg import Float64
from std_msgs.msg import String


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


#define service calls
services = []
for controller in controllers:
    services.append(rospy.ServiceProxy('/'+controller+'/torque_enable', TorqueEnable))


body_parts = {
    'front left leg': [1,2,3,4,5,6],
    'front right leg': [7,8,9,10,11,12],
    'back left leg': [13,14,15,16,17,18],
    'back right leg': [19,20,21,22,23,24]
}


# global variables used in callback functions
global positions
global button
global edit
global file
global name
global all_enabled
global left_arm_enabled
global right_arm_enabled
global head_enabled
global part
global motion_counter

# initial values for global variables
motion_counter = 0
all_enabled = 1
head_enabled = 1
left_arm_enabled = 1
right_arm_enabled = 1
positions = [0 for x in controllers]
file = None
button = 0
edit = 0
name = ""
part = ""

motion_pub = rospy.Publisher('play_motion', String)

# call back function for the keyboard/key topic subscribe
# h - help menu
# e - edit motion file
# n - new motion file
# c - exit motion file
# t - torque enable/disable


def get_motion_dir():
    homedir = os.environ['HOME']
    return (homedir + "/catkin_ws/src/the_walker/motions/")
   

def help_menu_generator(help=True):
    if help:
        my_string = 'motion saver v1.0 help\n'
    else:
        my_string = 'motion saver v1.0 - main menu\n'
    
    my_string += '- n = create a new motion file\n'
    my_string += '- e = edit a motion file\n'
    my_string += '- c = save and exit editting a motion file\n'
    my_string += '- p = play a motion file\n'
    my_string += '- t = turn on/off torque all servos\n'
    my_string += '- y = turn on/off torque a body part\n'
    my_string += '- h = help\n'

    return my_string


def torque_settings_generator():
    my_string = 'body Part Torque Setting:\n'

    for body_part in body_parts.keys():
        my_string += ('- ' + body_part + ' on\n')
        my_string += ('- ' + body_part + ' off\n')
    
    return my_string


def keyboard_capture(data):
    global button
    global edit
    global file
    global all_enabled
    global left_arm_enabled
    global right_arm_enabled
    global head_enabled
    global motion_counter
        
    button = data.code

    #help
    if button == 104:
        os.system('cls' if os.name == 'nt' else 'clear')
        print(help_menu_generator())
        name = raw_input("press enter to go back")
        
    #press on "p" to play motions
    if button == 112 and edit !=1:
        os.system('cls' if os.name == 'nt' else 'clear')
        name = ""
        name = raw_input("What motion do you want to play?: ")
        if name == "none":
            print "going back"
        else:
            motion_pub.publish(name)
        name = raw_input("press enter to go back")
                
    # press on "n" to create a new motion file
    if button == 110 and edit !=1:
        os.system('cls' if os.name == 'nt' else 'clear')
        name = ""
        name = raw_input("New motion name: ")
        name = get_motion_dir() + name + ".txt"
        file = open(name,"w+")
        file.close()
        print "new motion file is created"
        name = raw_input("press enter to go back")
        button = 0
                
    elif button == 110 and edit == 1:
        os.system('cls' if os.name == 'nt' else 'clear')
        print "you need to complete editing the previous motion file first - in order to do it press c"
        name = raw_input("press enter to go back")
        
        # press on "e" to edit a new motion file
        if button == 101 and edit !=1:
                
            os.system('cls' if os.name == 'nt' else 'clear')
                
            motion_directory = get_motion_dir()
                
            print "Your motions:"
            for root, dirs, files in os.walk(motion_directory):
                for f in files:
                    if f.endswith(".txt"):
                        print "- " + str(os.path.join(f))
                        
            name = ""
            name = raw_input("What motion do you want to edit?: ")
            if name == "none":
                print "nothing edited"
            else:
                name = get_motion_dir() + name + ".txt"
                        
                try:
                    file = open(name,"a")
                    print "motion file is opened for editting"
                    print "press on 'a' to add a new motion frame"
                    edit = 1
                    button = 0
                                
                except:
                    print "motion file doens't exist"
                                
        elif button == 101 and edit == 1:
            print "you need to complete editing the previous motion file first - in order to do it press c"
            name = raw_input("press enter to go back")
                
        # press on "a" to save the servo positions in a motion file
        if button == 97 and edit == 1:
            #print data.motor_states[0].position
            new_line = ""
            for position in positions:
                new_line = new_line + str(position) + ","
                                
            # ask user the delay time between motion steps/frames - add it to the end of each line
            delay = raw_input("delay(seconds):")
            new_line = new_line + str(delay)
            new_line = new_line + "\n"
            file.write(new_line)
            motion_counter = motion_counter + 1
            print str(motion_counter) + "new motion is added"
            print "press on 'a' to add a new motion frame press c to save exit and the motion file."
            button = 0
                
        # press on "c" to exit a motion file
        if button == 99 and edit == 1:
            os.system('cls' if os.name == 'nt' else 'clear')
            name = ""
            file.close()
            print "file is closed"
            edit = 0
            button = 0
            motion_counter = 0
                                        
        # press on "t" to exit a motion file
        if button == 116 and edit != 1:
            os.system('cls' if os.name == 'nt' else 'clear')
               
            # enable all servos
            if all_enabled == 0:
                for service in services:
                    service(1)
                all_enabled = 1
                print "torque is enabled"
                
            # disable all servos
            elif all_enabled == 1:
                for service in services:
                    service(0)
                all_enabled = 0
                print "torque is disabled"
                        
        if button == 121 and edit !=1:
            os.system('cls' if os.name == 'nt' else 'clear')
            print(torque_settings_generator())
               
            part = raw_input("type your choice:")

            if part.endswith(' on'):
                for body_part, servos in body_parts.iteritems():
                    if body_part in part:
                        for servo in servos:
                            services[servo](1)
                        print('{} is enabled'.format(body_part))
            elif part.endswith(' off'):
                for body_part, servos in body_parts.iteritems():
                    if body_part in part:
                        for servo in servos:
                            services[servo](0)
                        print('{} is disabled'.format(body_part))
            else:
                print "wrong parameter"
                        
            name = raw_input("press enter to go back")
        
        if edit != 1:
                os.system('cls' if os.name == 'nt' else 'clear')
                print(help_menu_generator(help=False))
                        
# callback funtions for reading joint locations
# updates the position[i] list
def set_position(data, position):
    global positions
    positions[position] = data.current_pos

callbacks = []

for i, controller in enumerate(controllers):
    callbacks.append(lambda data: set_position(data, i))


# main module for the motion saver node
# create a node and define callback funtions for topics
def motion_control():
        
        os.system('cls' if os.name == 'nt' else 'clear')
        print(help_menu_generator(help=False))
       
        rospy.init_node('motion_saver', anonymous=True)
        rospy.Subscriber('/keyboard/keydown', Key, keyboard_capture,queue_size=1)
        
        #callback funtion calls for the robot joints states
        subscribers = []
        for i, controller in enumerate(controllers):
            subscribers.append(rospy.Subscriber('/'+controller+'/state', JointState, callbacks[i]))
                                                                               
        rate = rospy.Rate(30)
        rospy.spin()
        
if __name__ == '__main__':
        try:
                motion_control()
        except rospy.ROSInterruptException:
                pass
