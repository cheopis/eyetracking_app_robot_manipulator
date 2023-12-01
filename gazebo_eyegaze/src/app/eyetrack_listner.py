#!/usr/bin/env python
import rospy
import tf
#import tkinter_testes.gaze_aplication as gaze_aplication
import pyautogui
from rt_gene.msg import MSG_BlinkList
import json

eye = True
head = False
sleep_time = 2

def eyeMovement(euler, data):
    if euler[2] > data[0]:
        pyautogui.press('right')
        rospy.sleep(sleep_time)
    elif euler[2] < data[1]:
        pyautogui.press('left') 
        rospy.sleep(sleep_time)
    elif euler[0] < data[3]:
        pyautogui.press('down')
        rospy.sleep(sleep_time)
    elif euler[0] > data[2]:
        pyautogui.press('up')   
        rospy.sleep(sleep_time) 

def headMovement(euler, data):
    if euler[2] > data[0]:
        pyautogui.press('right')
        rospy.sleep(sleep_time)
    elif euler[2] < data[1]:
        pyautogui.press('left') 
        rospy.sleep(sleep_time)
    elif euler[0] < data[3]:
        pyautogui.press('down')
        rospy.sleep(sleep_time)
    elif euler[0] > data[2]:
        pyautogui.press('up')
        rospy.sleep(sleep_time)

def blinkCallback(blink_msg):
    global blink
    global i
    blink = blink_msg.subjects[0].blink

    if blink:
        i = i + 1
    else:
        i = 0

    if i == 25:
        pyautogui.press('space')

    return blink

def mainTF():
    tf_listener = tf.TransformListener()

    global i
    i = 0

    blink_sub = rospy.Subscriber('subjects/blink', MSG_BlinkList, blinkCallback)

    file = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json", "r")
    content = file.read()
    data = json.loads(content)
    
    while not rospy.is_shutdown():
        try:
            if eye:
                (trans, rot) = tf_listener.lookupTransform('/gaze/world_gaze0', '/kinect2_link', rospy.Time(0))
                euler  =  tf.transformations.euler_from_quaternion(rot)
                eyeMovement(euler, data["rt_gene"]["eye"])
            elif head:
                (trans, rot) = tf_listener.lookupTransform('/gaze/head_pose_estimated0', '/kinect2_link', rospy.Time(0))
                euler  =  tf.transformations.euler_from_quaternion(rot)
                headMovement(euler, data["rt_gene"]["head"])
            else:
                print("No movement selected")
                break
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rospy.Rate(1.0).sleep() 

    file.close()

if __name__ == '__main__':
    rospy.init_node('gaze_aplication', anonymous=True)

    m_to_pixel = 3779.5275590551

    rate = rospy.Rate(10)

    #Thread(target = gaze_aplication.startApp).start() 
    #Thread(target = mainTF).start()
    mainTF()
       
