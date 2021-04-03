# Installation
*These steps have been tested on Ubuntu 18.04, making ROS Melodic work on newer versions of ubuntu might be possible but is a little bit trickier*

After installing Ubuntu 18.04:
1. Install ROS Melodic ([Official documentation](http://wiki.ros.org/melodic/Installation/Ubuntu))
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```
2. Install Eigen:
Download eigen version >3.4, open the folder in a terminal
```bash
sudo cp -r -t /usr/local/include/ Eigen/ unsupported/
```
3. Install Boost:
```bash
sudo apt install libboost-dev
```
4. Install python dependencies:
```bash
sudo apt install python3-pip python-pip
pip3 install rospkg dataclasses scipy numpy pyqtgraph
pip install scipy
```

5. If you have never installed another ROS package before:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

6. Install Simulator package and GUI plugin
Clone or download repository in your catkin workspace (usually ~/catkin_ws/src if you followed the ROS tutorials), then compile it:
```bash
cd ~/catkin_ws/src
git clone https://github.com/EPFLRocketTeam/real_time_simulator.git
git clone https://github.com/OTL/rqt_ez_publisher/tree/melodic-devel
cd ..
catkin_make
rqt --force-discover
```


Rviz config issue: The config of the rviz plugin in rqt will not load properly because it only stores the absolute path in the .perspective file.
This is fixed by writing your absolute path in GUI/rocket_GUI.perspective at the line:
```
"repr": "'*your real_time_simulator absolute path*/GUI/rocket_config.rviz'"
```

# Test
Once everything is installed, you can use the test_simu.sh bash script to simulate a basic flight and check that everything is properly working
```bash
roscd real_time_simulator/
./bash_scripts/test_simu.sh 
```
