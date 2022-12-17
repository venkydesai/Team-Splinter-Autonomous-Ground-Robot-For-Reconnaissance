sudo apt-get update && sudo apt-get upgrade
mkdir -p ~/ap_ws/src                # Make a new workspace 
cd ~/ap_ws/src                      # Navigate to the source space
echo "[+] Clonning github repos"
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper``` 
echo "[+] Done"
cd ..                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
echo "[+] Installing ros dependencies"
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
source devel/setup.bash # Source your ROS distro 
echo "source ~/ap_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "[+] Finished"