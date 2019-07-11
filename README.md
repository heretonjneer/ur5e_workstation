UR5e Workstation Package

** Adapted from nerve_workstation package which was generated for the Verizon 5g Challenge **
** Originally created to run nodes and launch files for working with the UR5e robot with some kinect cameras on one of the workstations and pedestals,
updated contents include launch files for using realsense cameras instead of kinect cameras specifically because of wrist mounted realsense camera removing the need for workstation mounted cameras/sensors for improved reliability and function

** Updated contents: contains launch files for running UR5e, linked to actual universal_robot and ur_modern_driver packages for bringup files and drivers, with solidworks-generated robotiq 2f 85 gripper with millibar and robotiq tcpip adapter and custom 3d printed realsense D435i camera wrist mount for accurate movement control and planning

** to run UR5e with robotiq 2f 85 gripper and wrist-mounter realsense d435i camera (imu functionality not included):

roslaunch ur5e_workstation ur5e_workstation.launch ip:=<ip.address.of.robot>

**To control the actual robot, press the power button on the top of the front of the ur5e pendant. Once the pendant has booted up, click the red button in the bottom left and presst ON to activate the robot and START to enable movement, then close the initialize window by clicking Exit in th ebottom left. in the top right is an icon that looks like the pendant, click this icon and select Remote Control, the only option. In the very top right is a menu button represented by three horizontal lines, click this and then click About. A window will pop up showing the robot's IP address, use this as the argument for launching the above launch file to get the robot going**
** If you need to manually move the robot around to reset a position or experiment with some joint configurations, simply navigate back to the top right of the pendant where you previously clicked on the image of the pendant to set Remote control. There is now an icon that represents remote controll in its place, click this and select Local control to regain control over the robot from the pendant ** 

** TODO: The urdf.xacro for the robotiq 2-finger gripper currently has all fixed joints, meaning the model will not reflect the fact that the gripper fingers can open and close. This will be adjusted later, but is currently not a necessity so it is being passed due to time constraints. 
** If you want to manually control a robotiq 2-finger gripper, run these nodes:

rosrun robotiq_2f_gripper_control Robotiq2FGripperTcpNode.py <your_gripper's_IP_address>
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

** This will require you to clone the robotiq package from github:
instructions below

** To get your gripper's IP address, go to the robotiq site and download the interface software for windows and plug it into your computer via USB (I am too tired to find this link for you right now, but I believe in you friend)
** If you belong to NERVE, the ip address is currently locked in at 10.10.10.42

** A link to the kinetic guide for using a robotiq 2-finger gripper (USB and TCP):
http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29

**TODO -put all custom packages on NERVE github and not on my github eventually once things are all cool**
**Necessary packages:**

**Universal Robot**
https://github.com/ros-industrial/universal_robot.git
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git

**UR Modern Driver**
**IT IS IMPORTANT YOU USE THE ONE FROM dniewinski AND NOT ros-industrial BECAUSE IT DOES NOT HAVE THE *e* VERSIONS**
https://github.com/dniewinski/ur_modern_driver
git clone -b kinetic-devel https://github.com/dniewinski/ur_modern_driver.git

**UR5e Joint Limited Robotiq 2f 85 Moveit Config**
**Custom Moveit! package for ur5e, will allow you to simulate the robot but is not heavily used since most files required for control of the actual robot are contained within the universal_robot and ur_modern_driver packages, gripper and adapters are in the robotiq_2f_85_full custom package and ur5e_workstation wraps the necessary launch files and nodes into a launch file (ur5e_workstation.launch) for you
https://github.com/flynn-nerve/ur5e_joint_limited_robotiq_2f_85_moveit_config
git clone -b master https://github.com/flynn-nerve/ur5e_joint_limited_robotiq_2f_85_moveit_config

**Robotiq**
**This is the solidworks-generated urdf package that you will need to run our current setup, this will not allow you to control the gripper, only represent it in rviz when running Moveit! stuff so the robot does not slam the gripper into objects and knows where to put the TCP**
https://github.com/flynn-nerve/robotiq_2f_85_full
git clone -b master https://github.com/flynn-nerve/robotiq_2f_85_full

**This is for controlling the gripper, not modeling it**
https://github.com/ros-industrial/robotiq
git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git
