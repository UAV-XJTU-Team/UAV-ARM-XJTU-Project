#!/usr/bin/env python  
# -*- coding: utf-8 -*  

import  os  
import  sys  
import  tty, termios   
import rospy  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String  
import select

# 全局变量  
cmd = Twist()  
pub = rospy.Publisher('keyboard', String, queue_size=10)  

def GetChar(Block=False):
  #初始化  
    rospy.init_node('keyboard_node')  
    rate = rospy.Rate(10)  

    #显示提示信息  
    print "Reading from keyboard"  
    print "Use WASD IJKL keys to control the UAV"  

    #读取按键循环  
    while not rospy.is_shutdown():  
        if Block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            ch=sys.stdin.read(1)
        else:
            ch='h'
        if ch=='q':
            break
        pub.publish(ch)  
        rate.sleep() 

# def test():
#     old_settings = termios.tcgetattr(sys.stdin)
#     tty.setcbreak(sys.stdin.fileno())

def keyboardLoop():  
    #初始化  
    rospy.init_node('keyboard_node')  
    rate = rospy.Rate(10)  

    #显示提示信息  
    print "Reading from keyboard"  
    print "Use WASD IJKL keys to control the UAV"  

    #读取按键循环  
    while not rospy.is_shutdown():  
        fd = sys.stdin.fileno()  
        old_settings = termios.tcgetattr(fd)  
        #不产生回显效果  
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO  
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            tty.setraw( fd )  
            ch = sys.stdin.read( 1 )  
        else:
            ch='h'
        if ch=='q':
            break
        pub.publish(ch)  
        rate.sleep()  
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  

if __name__ == '__main__':  
    try:  
        keyboardLoop()  
    except rospy.ROSInterruptException:  
        pass