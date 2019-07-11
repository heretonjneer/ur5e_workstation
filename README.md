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


*********************************************************************************************************************************************
**INSTRUCTIONS FOR INSTALLING REALSENSE CAMERAS**
**** Realsense package installation instructions ****

---- Setup ----
Unplug any realsense cameras before completing installation

cd <your_ws>/src

---- Download librealsense github repo ----
git clone -b  master https://github.com/IntelRealSense/librealsense.git

---- Install core packages required to build librealsense binaries ----
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

sudo apt-get install libglfw3-dev

---- Add server to list of repositories ----
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

---- Install libraries (and optional libraries) ----
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

---- Update and upgrade ----
sudo apt-get update && sudo apt-get upgrade

---- Install dependencies and then build workspace ----
rosdep install --from-paths src --ignore-src --rosdistro kinetic

**** Realsense cameras will not work if you do not run these following scripts (from the <your_ws>/src/librealsense directory) and give the ports permissions (udev rule settings) ****
cd librealsense

./scripts/setup_udev_rules.sh

./scripts/patch-realsense-ubuntu-lts.sh

---- Download realsense package (not realsense-ros package) ----

cd <your_ws>/src

git clone -b development https://github.com/doronhi/realsense.git

cd <your_ws>/src/realsense

git clone -b kinetic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git

cd ../..

---- Install dependencies and then build workspace ----
rosdep install --from-paths src --ignore-src --rosdistro kinetic

catkin build

---- Test packages if build completed ----
Plug in realsense camera

roslaunch realsense2_camera rs_camera.launch

rosrun rviz rviz

in rviz; add topic for image view from camera to check that camera is working

*********************************************************************************************************************************************
**ONLY FOLLOW THESE INSTRUCTIONS IF SOMETHING IS BROKEN**

---- If cameras will not work, check the libraries (librealsense-<stuff>, above) and if any say that they cannot be installed, follow these instructions:

sudo apt-get remove librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge

sudo rm -f /etc/apt/sources.list.d/realsense-public.list

sudo apt-get update

sudo apt-get install librealsense2-dkms

sudo apt-get install librealsense2-utils

sudo apt-get install librealsense2-dev

sudo apt-get install librealsense2-dbg

**ONLY DO THESE INSTRUCTIONS IF THE PREVIOUS ONES DO NOT WORK AND IT IS VERY BROKEN**
**** If this still does not work, you have to delete more and may have messed something up but it is fixable ****
**** Fair warning, this will make ros basically not work, it is not permanent, this is what I did when I broke everything ****

manually delete the librealsense and realsense packages

sudo apt-get remove librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge

sudo rm -f /etc/apt/sources.list.d/realsense-public.list

sudo apt-get remove --install-recommends linux-generic-lts-xenial xserver-xorg-core-lts-xenial xserver-xorg-lts-xenial xserver-xorg-video-all-lts-xenial xserver-xorg-input-all-lts-xenial libwayland-egl1-mesa-lts-xenial

sudo apt-get remove git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

sudo apt-get update && sudo apt-get upgrade

cd <your_ws>

rosdep install --from-paths src --ignore-src --rosdistro kinetic

catkin build

source ./devel/setup.bash

**** This should fix everything, you may find that a couple of packages you had installed from binaries are not working, just reinstall them and you should be good to go. at this point, go back to the start and redo the process. If everything is done correctly, you won't run into this issue ****


