#!/usr/bin/env python2 
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of myCobot, you can refer to these two variables for MyCobot initialization
import time
import RPi.GPIO as GPIO
import rospy 
from std_msgs.msg import Bool
# Initialize a MyCobot object
mc = MyCobot(PI_PORT, PI_BAUD)

G2_PIN=5
G5_PIN=6

GPIO.setmode(GPIO.BCM)
GPIO.setup(G2_PIN , GPIO.OUT)
GPIO.setup(G5_PIN , GPIO.OUT)
# Open sunction pump
def pump_off():
    GPIO.output(G2_PIN , GPIO.HIGH)
    GPIO.output(G5_PIN , GPIO.HIGH)  
 
    
# Stop sunction pump
def pump_on():
    GPIO.output(G2_PIN , GPIO.LOW)
    GPIO.output(G5_PIN , GPIO.LOW)      


#pump_off()
#time.sleep(3)
#pump_on()
#time.sleep(3)

def gripper_callback(data):
    if(data.data):
        pump_on()
    if(not data.data):
        pump_off()

# Defining the subscriber publisher
def talker():
    gripper_command_sub = rospy.Subscriber("suction_gripper_command",Bool,gripper_callback)
    rospy.init_node("suction_gripper_node")
    rate = rospy.Rate(100)
    rospy.spin()


if __name__ =='__main__':
    try:
        talker()
        
    except rospy.ROSInterruptException:
        pass
