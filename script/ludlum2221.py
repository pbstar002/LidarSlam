#!/usr/bin/env python3  

from __future__ import print_function

import roslib
import rospy
import serial
import std_msgs.msg

class Ludlum:
    def __init__(self):
        self.laser_port = rospy.get_param('/port','/dev/ttyUSB0')
        self.serial_ludlum = serial.Serial(self.laser_port)

        self.radiation_pub = rospy.Publisher('/radiation', std_msgs.msg.Int32,queue_size=1)
        self.actual_value = ''
        print('LUDLUM START')


    def update(self):
        self.actual_value = self.serial_ludlum.readline()
        # print(self.actual_value)
        temp_val = self.actual_value.splitlines()
        str_val = temp_val[0].decode("utf-8")
        val = std_msgs.msg.Int32()

        val.data = int(str_val.split('R',1)[1])
        self.radiation_pub.publish(val)
        # print(val)




#---------------
if __name__=='__main__':
    try:
        rospy.init_node('Ludlum_node')

        ludlum = Ludlum()  
        
        rate = 100
        ros_rate = rospy.Rate(rate)
        
        while not rospy.is_shutdown():
            ludlum.update()
            # rospy.spin()
            ros_rate.sleep()

    except rospy.ROSInterruptException:
        print ('[Ludlum_node] Interrupt.',file=sys.stderr)

