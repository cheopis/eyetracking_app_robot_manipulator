#!/usr/bin/env python
from threading import Thread
import rospy
import json

import get_yolo
import tf_listner
import tkinter_app
#import tkinter_testes.gaze_aplication as gaze_aplication

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


if __name__ == '__main__':
    rospy.init_node('gaze_aplication', anonymous=True)
    rate = rospy.Rate(10)
    
    #Thread(target = gaze_aplication.startApp).start()
    with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'r') as f:
        data = json.load(f)
        if data["gazebo"]["enabled"]:
            print(bcolors.OKCYAN + '============  gazebo enabled  ============ ' + bcolors.ENDC)
            if data["yolo"]["enabled"]:
                print(bcolors.OKCYAN + '============  yolo enabled  ============ ' + bcolors.ENDC)
                Thread(target = tkinter_app.startApp).start()
                get_yolo.main()
            else: 
                print(bcolors.WARNING + '============  yolo disabled  ============ ' + bcolors.ENDC)

        else: 
            print(bcolors.WARNING + '============  gazebo disabled  ============ ' + bcolors.ENDC)

        if data["rt_gene"]["enabled"]:
            print(bcolors.OKCYAN + '============  rt_gene enabled  ============ ' + bcolors.ENDC)
            Thread(target = tkinter_app.startApp).start()
            tf_listner.mainTF()    
        else: 
            print(bcolors.WARNING + '============  rt_gene disabled  ============ ' + bcolors.ENDC)
