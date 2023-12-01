#!/usr/bin/env python2

#app libraries
from tkinter import *
from tkinter.ttk import *
from tkinter import Tk, Canvas, Frame, Button, Label, Entry
from tkinter import BOTH, W, NW, TOP, FLAT
from PIL import Image, ImageTk
import tkinter 
import os
import os.path
import json
import get_yolo as get_yolo
import time
from pathlib import Path

UPPER_CAMERA = True
CAMERA_HEIGHT = 800
CAMERA_WIDTH = 800
CALIBRATION_BOX = 0.0008196721

class Application(Frame):
    def __init__(self, master = None):
        Frame.__init__(self)
        self.parent = master
        self.initUI()
        
    def initUI(self):
        self.parent.title("Layout Test")
        self.config(bg = '#F0F0F0')
    
        #create buttons
        self.curBut = [-1,-1]
        self.buttonL = [[]]
        self.entry = Entry(self, width=30)
        self.entry.grid(row = 3, column = 0, columnspan=2)
        self.createWidgets()

        #self.entry.pack()
        self.inicio = 0

        #Generate images
        self.getImages()
        self.canvas = Label(self, relief = FLAT, background = "#D2D2D2", width = 500, 
                             height = 500, image = self.items["img"][self.index])
        self.canvas.grid(row = 0, column = 2, columnspan=2, rowspan = 4)
        self.pack()

    # creating the buttons
    def createWidgets(self):
        but = tkinter.Button(self, text=" Prev ", width=10, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.prevItem)
        self.buttonL[0].append(but)
        but.grid(row = 0, column=0)

        but = tkinter.Button(self, text=" Next ", width=10, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=12,
                        pady=4, bd=4, command = self.nextItem)
        self.buttonL[0].append(but)
        but.grid(row = 0, column=1)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Select ", width=22, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.selectItem)
        self.buttonL[1].append(but)
        but.grid(row = 1, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Quit ", width=22, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.quit)
        self.buttonL[2].append(but)
        but.grid(row = 2, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])  


    

    # creating the buttons
    def createWidgets2(self):
        but = tkinter.Button(self, text=" Start ", width=22, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.start)
        self.buttonL[0].append(but)
        but.grid(row = 0, column=0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Prev ", width=10, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.prevItem)
        self.buttonL[1].append(but)
        but.grid(row = 1, column=0)
        

        but = tkinter.Button(self, text=" Next ", width=10, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=12,
                        pady=4, bd=4, command = self.nextItem)
        self.buttonL[1].append(but)
        but.grid(row = 1, column=1)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Select ", width=22, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.selectItem)
        self.buttonL[2].append(but)
        but.grid(row = 2, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Quit ", width=22, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.quit)
        self.buttonL[3].append(but)
        but.grid(row = 3, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

    def leftKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')

        elif self.curBut[0] == 0:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        else:
            pass

    def rightKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 0:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [0,1]
            self.buttonL[0][1].configure(highlightbackground='red')
        else:
            pass

    def upKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 0:
            pass
        else:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [(self.curBut[0]-1), 0]
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='red')

    def downKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 2:
            pass
        else:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [(self.curBut[0]+1), 0]
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='red')

    def select(self,value, x, y):
        if self.curBut != [-1,-1]:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
        self.curBut[:] = [x,y]
        self.buttonL[x][y].configure(highlightbackground='red')
        self.entry.insert(tkinter.END, value)

    # eyegaze mouse functions
    def invoke_button(self,event):
        self.buttonL[self.curBut[0]][self.curBut[1]].invoke()

    def getImages(self):
        f = '/home/ariele/catkin_ws/src/gazebo_eyegaze/images'
        self.items = {"name":[],
                      "img": []}
        with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'r') as fjson:
            data = json.load(fjson)
            item = data["app"]["items"]    

        for file in os.listdir(f):
            f_img = f+"/"+file
            if file[0:-4] in item:
                self.items["name"].append(file[0:-4])
                img = Image.open(f_img)
                img = img.resize((500, 400))
                self.items["img"].append(ImageTk.PhotoImage(img))
        print(self.items["name"])
        self.index = 0

    # select the item and send the information to robot
    def start(self):
        self.inicio = time.time()

    def selectItem(self):
        fim = time.time()

        self.entry.delete(0, 'end')
        with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'r') as f:
            data = json.load(f)
        if self.items["name"][self.index] in data["yolo"]["all"]:
            self.entry.insert(tkinter.END, 'Selected: ' + self.items["name"][self.index])
            
            data["app"]["selected"] = self.items["name"][self.index]
            with open('/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json', 'w') as json_file:
                json.dump(data, json_file, indent=4)

        else:
            self.entry.insert(tkinter.END, self.items["name"][self.index] + ' not found')

        file = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/keyboard_data.txt", "a")
        file.writelines([str(fim - self.inicio),"\n"])
        file.close()

    def prevItem(self):
        self.index -= 1
        if self.index < 0:
            self.index = len(self.items["img"]) - 1
        self.canvas.configure(image = self.items["img"][self.index])

    def nextItem(self):
        self.index += 1
        if self.index >= len(self.items["img"]):
            self.index = 0
        self.canvas.configure(image = self.items["img"][self.index])

    def atualize(self,e=None):
        self.getImages()
        self.canvas = Label(self, relief = FLAT, background = "#D2D2D2", width = 500, 
                             height = 500, image = self.items["img"][self.index])
        self.canvas.grid(row = 0, column = 2, columnspan=2, rowspan = 4)
        self.pack()

def startApp():
    root = Tk()
    root.geometry('800x500+10+50')
    #root.overrideredirect(True)
    app = Application(root)

    # This will bind eyegaze events to the tkinter
    # toplevel which will define the mouse clicks
    root.bind('<Left>', lambda e: app.leftKey(e))
    root.bind('<Right>', lambda e: app.rightKey(e))
    root.bind('<Up>', lambda e: app.upKey(e))
    root.bind('<Down>', lambda e: app.downKey(e))
    root.bind('<space>', lambda e: app.invoke_button(e))
    root.bind("<F5>", app.atualize)

    app.mainloop() 

if __name__ == '__main__':
    startApp()
      
