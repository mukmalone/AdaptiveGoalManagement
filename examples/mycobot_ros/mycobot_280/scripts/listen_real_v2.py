#!/usr/bin/env python2
# license removed for brevity
import time
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.srv import GetAngles
from pymycobot import PI_PORT, PI_BAUD 
from pymycobot.genre import Angle
from pymycobot.mycobot import MyCobot

mc = MyCobot(PI_PORT, PI_BAUD)
def talker():
    rospy.loginfo("start ...")
    rospy.init_node("real_listener", anonymous=True)
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    
    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
            "arm1_joint",
            "arm2_joint",
            "arm3_joint",
            "arm4_joint",
            "arm5_joint",
            "arm6_joint",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []



    rospy.loginfo("start loop ...")
    while not rospy.is_shutdown():
    
        radians_list = mc.get_angles()
        radians_list = radians_list
        rospy.loginfo("res: {}".format(radians_list))

        # publish angles.
        joint_state_send.header.stamp = rospy.Time.now()
        joint_state_send.position = radians_list
        pub.publish(joint_state_send)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
