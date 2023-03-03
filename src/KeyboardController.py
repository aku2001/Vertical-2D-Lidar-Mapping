#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist

class KeyboardController:
    
    def __init__(self) -> None:
        
        self.enableKeyboard = False
        print("Keyboard Disabled. To Activate and Disactivate Press 'E'")

        # Set Keys
        self.enableKey = 'E'
        self.stopKey = 'x'

        # keys contains [x,z]
        self.movementKeys = {'w':[1,0],'a':[0,1],'s':[-1,0],'d':[0,-1],'x':[0,0],keyboard.Key.up:[1,0],
        keyboard.Key.down:[-1,0],keyboard.Key.right:[0,-1],keyboard.Key.left:[0,1]}

        # Set Speed
        self.linSpeed = 0.5
        self.angSpeed = 1


        # ...or, in a non-blocking fashion:
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        # Start program
        self.initRos()
        self.publishCmd()
    
    def publishCmd(self):

        while not rospy.is_shutdown():
            if(self.enableKeyboard):
                self.cmdPublisher.publish(self.cmdVelMsg)
            
            self.rate.sleep()
    
    def initRos(self):
        rospy.init_node("keyboadController")
        self.cmdPublisher = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        self.cmdVelMsg = Twist()

        # set rate
        self.rate = rospy.Rate(10)

    def on_press(self,key):

        if(not isinstance(key,keyboard.Key)):
            key = key.char
        
        if(key in self.movementKeys):
            val = self.movementKeys[key]
            self.cmdVelMsg.linear.x = self.linSpeed * val[0]
            self.cmdVelMsg.angular.z = self.angSpeed * val[1]

        
        elif(key == self.enableKey):
            self.enableKeyboard = not self.enableKeyboard

            if(self.enableKeyboard):
                print("Keyboard Enabled")
            else:
                print("Keyboard Disabled")
        

    def on_release(self,key):
        
        val = self.movementKeys[self.stopKey]
        self.cmdVelMsg.linear.x = self.linSpeed * val[0]
        self.cmdVelMsg.angular.z = self.angSpeed * val[1]


# if __name__ == "__main__":

KeyboardController()