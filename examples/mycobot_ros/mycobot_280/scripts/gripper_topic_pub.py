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


# Defining the subscriber publisher
def talker():
    rospy.init_node("gripper_node")
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        
        gripper_value= mc.get_gripper_value()
        print(gripper_value)
        pub.publish(gripper_value) 
        rate.sleep()

if __name__ =='__main__':
    try:
        talker()
        
    except rospy.ROSInterruptException:
        pass