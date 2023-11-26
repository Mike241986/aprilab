#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

from geometry_msgs.msg import PointStamped

from dynamic_reconfigure.server import Server as DynamicReconfigServer

from dynamic_reconfig.cfg import dynamic_reconfig_testConfig
#this from dynamic_reconfig.cfg is the package name and the .cfg inside the package, 
#so it is from "dynamic_reconfig" package .cfg part, import dynamic_reconfig_test(Config), Config needs to be appended after the node name
#refer to the file inside the dynamic_reconfig package to see details
#reference: https://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
#reference: https://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

class test_controller():
    def __init__(self):
        rospy.init_node('test_controller')
        self.pub = rospy.Publisher("/rudder_angle",Float32, queue_size = 10)
        self.ps_sub = rospy.Subscriber("nmea_lla", PointStamped, self.stanleyControl)
        self.config = DynamicReconfigServer(dynamic_reconfig_testConfig, self.reconfig)
        self.x = 0
        self.y = 0

        self.int = 0
        self.double = 0
        self.bool = True

    def stanleyControl(self, message):
        self.x = message.point.x
        self.y = message.point.y

        print(f'x = {self.x}, y = {self.y}')
        print()
        self.pub.publish(Float32(1))

    def reconfig(self, reconfig, level):
        #the reconfig is a dictionary like object, so to access them, use similar syntax as matrix
        integer = reconfig['int_param']
        double = reconfig['double_param']
        boolean = reconfig['bool_param']

        print(f'Integer: {integer}, Double: {double}, boolean: {boolean}')
        self.int = reconfig['int_param']
        self.double = reconfig['double_param']
        self.bool = reconfig['bool_param']

        #seems like we need this line "return reconfig" to avoid any errors
        return reconfig


def main(): 
    test_controller()
    rospy.spin()

main()

#this method of initializing the controller works 


    

