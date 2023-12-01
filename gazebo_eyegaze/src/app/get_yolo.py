#!/usr/bin/env python2  
import rospy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
import tkinter_app
from threading import Thread, Event
import json
import pick_and_place as pick

UPPER_CAMERA = True
CAMERA_HEIGHT = 800
CAMERA_WIDTH = 800
CALIBRATION_BOX = 0.0008196721 / 0.8

f = '/home/ariele/catkin_ws'

class yolo_subscriber:
    def __init__(self):
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxCallback)
        self.count_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.countCallback)

    def boxCallback(self,data):
        self.boxes = data.bounding_boxes
        self.getBoxParam()

    def countCallback(self,data):
        self.count = data.count

    def getItemPosition(self, i):
        x_med = 240 - (self.boxes[i].ymax + self.boxes[i].ymin)/2
        y_med = 320 - (self.boxes[i].xmax + self.boxes[i].xmin)/2

        if UPPER_CAMERA:
            x_item = CALIBRATION_BOX*x_med + 0.58
            y_item = CALIBRATION_BOX*y_med + 0.0
            z_item = 0.43 
        
        item_pose = (x_item,y_item,z_item)
        return item_pose

    def getBoxParam(self):
        items = []
        all = []
        try:
            with open(f+'/src/gazebo_eyegaze/src/app/data.json', "r") as json_file:
                data = json.load(json_file)
                for i in range(self.count):
                    if self.boxes[i].Class in data["app"]["enabled"]:
                        all.append(self.boxes[i].Class)
                        item = {"name": self.boxes[i].Class + "_0" + str(i),
                                "type": self.boxes[i].Class,
                                "pos": self.getItemPosition(i)}
                        items.append(item)

                data["app"]["items"] = all
                data["yolo"]["items"] = items

            with open(f+'/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
                json.dump(data, json_file, indent=4)
        except:
            pass

    def selectedItem(self):
        with open(f+'/src/gazebo_eyegaze/src/app/data.json', 'r') as json_file:
            data = json.load(json_file)
            if not data["app"]["selected"]: 
                return
            else:
                name = data["app"]["selected"]
                data["app"]["selected"] = False
                data["yolo"]["selected"] = name
                with open(f+'/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
                    json.dump(data, json_file, indent=4)
        
        try:
            for i in range(self.count):
                if self.boxes[i].Class == name:
                    item_pose = self.getItemPosition(i)
                    pick.MoveItPanda(item_pose, name + "_0" + str(i))
        except:
            return

def main():
    box = yolo_subscriber()
    while not rospy.is_shutdown():
        #box.getBoxParam()
        box.selectedItem()
        rospy.Rate(1.0).sleep() 


if __name__ == '__main__':
    Thread(target = tkinter_app.startApp).start() 
    #Thread(target = listener).start()
    main()
    
