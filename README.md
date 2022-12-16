
# Team Splinter Autonomous Ground Robot For Reconnaissance using RRT Exploration.
This repo contains all the working code and procudure for autonomous exlporation and SLAM. Below are the steps to successfuly build the workspace. Foe testing purpose Apriltags were placed in the enironment as victims and the main objective of the robot was to create an occupancy grid map of the environment and detect all the Apriltags in the environment along with its pose in the environment.
### System setup
> 1. Turtlebot3 platform?
>    
>    * [x]  Burger
>    * [ ]  Waffle
>    * [ ]  Waffle Pi
> 2. ROS working on SBC and Remote PC?
>    
>    * [ ]  ROS 1 Kinetic Kame
>    * [ ]  ROS 1 Melodic Morenia
>    * [x]  ROS 1 Noetic Ninjemys
>    * [ ]  ROS 2 Dashing Diademata
>    * [ ]  ROS 2 Eloquent Elusor
>    * [ ]  ROS 2 Foxy Fitzroy
>    * [ ]  etc (Please specify your ROS Version here)
> 3. SBC(Single Board Computer) working on TurtleBot3?
>    
>    * [ ]  Intel Joule 570x
>    * [ ]  Raspberry Pi 3B+
>    * [x]  Raspberry Pi 4
>    * [ ]  etc (Please specify your SBC here)
> 4. OS installed on SBC?
>    
>    * [ ]  Raspbian distributed by ROBOTIS
>    * [ ]  Ubuntu MATE (16.04/18.04/20.04)
>    * [x]  Ubuntu preinstalled server (18.04/20.04)
> 5. OS installed on Remote PC?
>    
>    * [ ]  Ubuntu 16.04 LTS 
>    * [ ]  Ubuntu 18.04 LTS 
>    * [x]  Ubuntu 20.04 LTS (Focal Fossa)
>    * [ ]  Windows 10
>    * [ ]  MAC OS X 

### Getting started with turtlebot.
For this setup we have used **ROS Noetic** and **Ubuntu 20.04** in both Remote PC (your Laptop) and SBC (Single Board Computer - Rpi).
Follow the quick start guide for building the necessary packages in both the computers.
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Up until now you would have completly setup the robot and you PC. Now we will start installing required packages.

### Installing RRT Exploration package.
The official rrt_exploration package was built was **Ros Kinetic**, hence we will not be using that default one. We will clone the ported repo. Follow below steps to install the package.
http://wiki.ros.org/rrt_exploration

```console
cd ~/catkin_ws/src # Navigate to the source space
git clone https://github.com/prabinrath/rrt_exploration.git  # Clone RRT library
cd .. # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin_make # Build all packages in the workspace 
source devel/setup.bash # Source your ROS distro 
```

### Installing AprilTag Ros Package.
For apriltag detection, we will be using apriltag_ros package which gives pose and tag id of the detected tags in the environment. The apriltag_ros may sometimes cause issue with the installed ros packages, hence for this reason we will be creating a separate workspace specifically for apriltag *This is a better option than to create everything in one workspace.*
https://github.com/AprilRobotics/apriltag_ros
```console
mkdir -p ~/ap_ws/src                # Make a new workspace 
cd ~/ap_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
cd ..                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
source devel/setup.bash # Source your ROS distro 
```
**Add the setup files in bashrc file of your pc**
```console
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/ap_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing Exlpore lite package (Optional, Just for benchmarking).
**ROS Noetic** uses explore lite package for greedy based frontier exploration. More details can be found here http://wiki.ros.org/explore_lite
```console
cd ~/catkin_ws/src 
git clone https://github.com/hrnr/m-explore.git
cd .. 
rosdep install --from-paths src --ignore-src -r -y  
catkin_make 
source devel/setup.bash
```
There are certain parameters in explore_lite which you can change depending on your exvironment. For our case setting the below parameter's value became fruitfull. *Only change if the default exlpore_lite is not working inyour environment*
```console
gain_scale = 1
potential_scale = 4
min_frontier_size = 0.4 
transform_tolerance = 2 #Maximum allowed tf tolerance in seconds. If the difference between the tf from map and robot is greater than transform_tolerance than the move base will stop the robot.
```
### Tunning the Turtlebot.
As default turtlebot bringup file only initialises the *OpenCR and Sensors* not the camera. Hence we need to change the default launch file so that we don't have to run rpi camera launch file separately.
```console
sudo ssh -l pi <IP_OF_RPI>
cd ~/catkin_ws/src/turtlebot3/turtlebot3_bringup/launch
nano turtlebot3_robot.launch
```
On line 14 change the group condition to *burger* as in this project we are using turtlebot3 burger.
There might be instances where you will face time synchronisation errors. Please follow below steps on both RPC and SBC.
```console
sudo apt-get install ntpdate
sudo ntpdate ntp.ubuntu.com
```
```console
systemctl mask systemd-networkd-wait-online.service #SBC will start even though it is not online
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target #Disable hybernation and sleep
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
sudo apt-get update && sudo apt-get upgrade
```

