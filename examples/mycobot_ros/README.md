# mycobot_ros 

Modified mycobot_ros pkg for myCobot Pi

This is a forked project of [the official repository](https://github.com/elephantrobotics/mycobot_ros)

Clone the pkg into the raspberry pi's local workspace:
```bash
git clone -b mycobot-pi https://github.com/QibiTechInc/mycobot_ros.git
```
Launch the following to test the system, you should be able to see myCobot Pi in Rviz with a visual interface that by changing the joints value the robot's actual joint state would change
```bash 
roslaunch mycobot_280 simple_gui.launch 
```

## Having the parallel gripper in ROS
Launch the following pkg:
```bash
roslaunch mycobot_280 gripper.launch  
```
This will runs the necessary codes for having the gripper in ROS. For commanding the gripper to open, use the `/gripper_command` topic. if you publish 0 and 100 (std_msgs::Float64), the gripper will close or open completely respectively. The numbers in between might not work exactly. 
You can get the the gripper current status (a number between 0 and 100) through the topic `/gripper_status`. Note that this topic sometimes publishes -1 instead of the real value, which seems to be a problem from the robot's API (specially, if you are reading the joint_angles, that is to say, you have launched the mycobot.launch , or the MoveIt pkgs before)

## Having the suction gripper in ROS

First connect the G2 and G5 node to the 5th and 6th pin of the raspberry Pi. Then run the following command:
```bash
rosrun mycobot_280 suction_gripper.py 
```
By running this node and publising std_msgs::Bool type msgs to `suction_gripper_command` topic you can turn on or off the suction gripper. True and False mean turning the suction pump on and off respectively. If you wish to change the pin numbers, change the G2_PIN and G5_PIN to the value of the pin number that you wish to have. 

<div style="text-align:center"><video controls width="300" alt="video_2021-11-13_17-12-43.mp4 (3.2 MB)" src="https://esa-storage-tokyo.s3-ap-northeast-1.amazonaws.com/uploads/production/attachments/12680/2021/11/13/116092/c6c2ff8b-d2f5-4d3b-877b-f93d00ae422d.mp4"></video></div>
