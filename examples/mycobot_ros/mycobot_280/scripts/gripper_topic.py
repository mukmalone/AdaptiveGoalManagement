#!/usr/bin/env python2
from pymycobot import mycobot
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from pymycobot import PI_PORT, PI_BAUD 
from pymycobot.mycobot import MyCobot
from mycobot_communication.srv import GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus

mc = MyCobot(PI_PORT,PI_BAUD)

pub = rospy.Publisher("gripper_status",Float64,queue_size=10)
def gripper_callback(data):
    mc.set_gripper_value(data.data,50)
    gripper_value= mc.get_gripper_value()
    pub.publish(gripper_value)  
    print('received the message')
    


# Defining the subscriber publisher
def talker():
    gripper_command_sub = rospy.Subscriber("gripper_command",Int8,gripper_callback)
    i=1
    rospy.init_node("gripper_node")
    rate = rospy.Rate(10)
    rospy.spin()
        

if __name__ =='__main__':
    try:
        talker()
        
    except rospy.ROSInterruptException:
        pass