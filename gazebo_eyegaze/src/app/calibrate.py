#!/usr/bin/env python
import rospy
import tf
from six.moves import input
import os
import numpy as np
from playsound import playsound
import json

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def aquire_Data(type):
    tf_listener = tf.TransformListener()
    count = 0
    euler = 0
    while count < 25 and not rospy.is_shutdown():
        try:
            if type == 'eye':
                (trans, rot) = tf_listener.lookupTransform('/gaze/world_gaze0', '/kinect2_link', rospy.Time(0))
                eye_euler  =  tf.transformations.euler_from_quaternion(rot)
                euler += np.array(eye_euler)
            elif type == 'head':
                (trans, rot) = tf_listener.lookupTransform('gaze/head_pose_estimated0', '/kinect2_link', rospy.Time(0))
                head_euler  =  tf.transformations.euler_from_quaternion(rot)
                euler += np.array(head_euler)

            else:
                print("No movement selected")
                break
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        count += 1
        rospy.Rate(1.0).sleep() 
    
    return euler/count

def calibrate(type, duration, freq):
    if type == "eye":
        text = "Look"
        i = 2
    elif type == "head":
        text = "Move your head"
        i = 2

    path_sound = 'catkin_ws/src/gazebo_eyegaze/src/app/sound.mp3'

    print(bcolors.OKCYAN + '======== calibrating ' + type + ' tracking ========')
    print(bcolors.OKCYAN + text + ' in the direction requested until the beep sounds')

    input( text + ' up and press enter')
    up = aquire_Data(type)[0]
    #os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
    playsound(path_sound)

    input( text + ' down and press enter')
    down =  aquire_Data(type)[0]
    #os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
    playsound(path_sound)

    input( text + ' to the right and press enter')
    right =  aquire_Data(type)[i]
    #os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
    playsound(path_sound)
    
    input( text + ' to the left and press enter')
    left =  aquire_Data(type)[i]
    #os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
    playsound(path_sound)

    return [right, left, up, down]
    
def mainTF():
    rospy.init_node('gaze_aplication', anonymous=True)

    # Calibrating Eye tracking
   # eye = calibrate("eye", 0.2, 440)
    # Calibrating Head tracking
    head = calibrate("head", 0.2, 440)
    
    file = open("catkin_ws/src/gazebo_eyegaze/src/app/data.json", "r")
    content = file.read()
    data = json.loads(content)
    #data["rt_gene"]["eye"] = eye
    data["rt_gene"]["head"] = head

    with open('catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
        json.dump(data, json_file, indent=4)

    #file.close()


if __name__ == '__main__':
    #Thread(target = gaze_aplication.startApp).start() 
    #Thread(target = mainTF).start()
    mainTF()