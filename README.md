

# Autonomous Ground Robot For Reconnaissance using RRT Exploration.
This repo contains all the working codes and procedures for autonomous exploration and SLAM. Below are the steps to successfully build the workspace. *~Team Splintler*

For testing purposes, Apriltags were placed in the environment as victims and the main objective of the robot was to create an occupancy grid map of the environment and detect all the Apriltags along with their pose in the environment.
### System setup
****
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
> 5. OS installed on the Remote PC?
>    
>    * [ ]  Ubuntu 16.04 LTS 
>    * [ ]  Ubuntu 18.04 LTS 
>    * [x]  Ubuntu 20.04 LTS (Focal Fossa)
>    * [ ]  Windows 10
>    * [ ]  MAC OS X 

### Getting started with turtlebot.
****
For this setup we have used **ROS Noetic** and **Ubuntu 20.04** in both Remote PC (your Laptop) and SBC (Single Board Computer - Rpi).
Follow the quick start guide for building the necessary packages on both computers.
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Up until now, you would have completely set up the robot and your PC. Now we will start installing the required packages.

### Installing RRT Exploration package.
****
The official rrt_exploration package was built was **Ros Kinetic**, hence we will not be using that default one. We will clone the ported repo. Follow the below steps to install the package.
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
****
For apriltag detection, we will be using the apriltag_ros package which gives the pose and tag id of the detected tags in the environment. The apriltag_ros may sometimes cause an issue with the installed ros packages, hence for this reason we will be creating a separate workspace specifically for apriltag *This is a better option than creating everything in one workspace.*

Alternatively you can run the apriltag_ws_build shell script from this repo by running ./apriltag_ws_build.sh
https://github.com/AprilRobotics/apriltag_ros
****
On RPC
```console
mkdir -p ~/ap_ws/src                # Make a new workspace 
cd ~/ap_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper``` 
cd ..                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
source devel/setup.bash # Source your ROS distro 
```
**Add the setup files in the bashrc file of your pc**
```console
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/ap_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Download the tag_detec.py file from this repo and paste it in apriltag_ros/apriltag_ros/src and run catkin build.
### Installing Explore lite package (Optional, Just for benchmarking).
****
**ROS Noetic** uses explore lite package for greedy-based frontier exploration. More details can be found here http://wiki.ros.org/explore_lite
```console
cd ~/catkin_ws/src 
git clone https://github.com/hrnr/m-explore.git
cd .. 
rosdep install --from-paths src --ignore-src -r -y  
catkin_make 
source devel/setup.bash
```
There are certain parameters in explore_lite that you can change depending on your environment. For our case setting the below parameter's value became fruitful. *Only change if the default exlpore_lite is not working on your environment*
```console
gain_scale = 1
potential_scale = 4
min_frontier_size = 0.4 
transform_tolerance = 2 #Maximum allowed tf tolerance in seconds. If the difference between the tf from the map and the robot is greater than transform_tolerance then the move base will stop the robot.
```
### Tunning the Turtlebot.
****
As default turtle bot bringup file only initializes the *OpenCR and Sensors* not the camera. Hence we need to change the default launch file so that we don't have to run the rpi camera launch file separately.
```console
sudo ssh -l pi <IP_OF_RPI>
cd ~/catkin_ws/src/turtlebot3/turtlebot3_bringup/launch
nano turtlebot3_robot.launch
```
On line 14 change the group condition to *burger* as in this project we are using turtlebot3 burger.
There might be instances where you will face time synchronization errors. Please follow the below steps on both RPC and SBC.
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
## Testing the setup.
****
Follow the below steps to check if the setup os working properly or not. *Note: Don't forget to run roscore before bringing up the robot from SBC.
Your directory setup might look like this.
```console
ap_ws/src/
├── apriltag
│   ├── CMake
│   ├── common
│   └── example
└── apriltag_ros
    └── apriltag_ros
        ├── config
        ├── docs
        ├── include
        │   └── apriltag_ros
        ├── launch
        ├── msg
        ├── scripts
        ├── src
        └── srv
        
15 directories
```
**On SBC**
****
```console
sudo ssh -l pi <IP_OF_RPI>
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
**On RPC (Remote PC)**
****
--> Launch all the roslaunch files in different terminals.
```console
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping #We are using gmapping just for the sake of simplicity and lightweight. You can use a cartographer also for better robot localization.
roslaunch turtlebot3_navigation move_base.launch
roslaunch explore_lite explore.launch
```
If everything is installed as required then your robot should start moving autonomously into the environment and you will be able to see the map generated in the Rviz window on your PC.
*Remember as we are using Rpi 3 B+ 1GB, there is nothing that should be installed on SBC unless required as this will increase the system load for Rpi.*

After the exploration is completed an info message will be printed on the move_base terminal as "Goal Reached". This means there are no more frontiers and no unexplored regions is there. Now you can save your map as a PGM file by running the below command on RPC.
```console
rosrun map_server map_saver -f dry_run_map
```
# Initialising...
So far we have installed all the required dependencies and packages required for this project. Now it's time to test it all together. For aprilatg id and pose detection getting published we have created an algorithm for that in whcih the apriltag_tracking node will keep a note on detected apriltags and their pose and write those in a unique txt file so that if unexpectedly the node gets crashed then we can have the data for each apriltag.
For exploring the robot using RRT exploration follow the same steps as you did in testing the setup but instead of launching explore_lite we will be launching rrt_exploration.
*Follow the below sequence of commands as there might be tf costmap synchronisation errors due to the tag_detect.py, this optimisation is left for future work.

Also for rrt_exlporationn you need to provide in total of 5 points which denotes the global and local planner to explore the tree in that manner. 
*Remember this step is extremely important for rrt based exploration because if the points are selected in different pattern or different size than the birdeye view of the map then it will cause unexpected results*
You can find more details in the report in this repo and here http://wiki.ros.org/rrt_exploration/Tutorials/singleRobot

**On RPC**
****
Run all the launch file in different terminals.
```console
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch apriltag_ros continous_detecttion.launch
rosrun apriltag_ros tag_detect.py
roslaunch turtlebot3_navigation move_base.launch
roslaunch rrt_exploration single.launch #select 5 points and the robot should start moving.
```
Below is the output of the run. If you notice that the robot doesnot move in a continous manner, this was caused by synchronisation error and tag_detect.py code.
https://northeastern-my.sharepoint.com/personal/desai_ven_northeastern_edu/_layouts/15/onedrive.aspx?ga=1&id=%2Fpersonal%2Fdesai%5Fven%5Fnortheastern%5Fedu%2FDocuments%2FSplinter
Youtube - https://youtu.be/gg17nGIT8wM
