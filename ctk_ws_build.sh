sudo apt-get install terminator
cd ~/catkin_ws/src 
git clone https://github.com/hrnr/m-explore.git
git clone https://github.com/prabinrath/rrt_exploration.git
cd .. 
rosdep install --from-paths src --ignore-src -r -y  
catkin build
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc