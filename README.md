# Installation
*These steps have been tested on Ubuntu 18.04, making ROS Melodic work on newer versions of ubuntu might be possible but is a little bit trickier*

After installing Ubuntu 18.04:
1. Install ROS Melodic ([Official documentation](wiki.ros.org/melodic/Installation/Ubuntu))
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
2. Install Eigen
Download eigen version >3.4, open the folder in a terminal
```bash
sudo cp -r -t /usr/local/include/ Eigen/ unsupported/
```
3. Install Boost
```bash
sudo apt install libboost-dev
```
4. Install python dependencies:
```bash
sudo apt install python3-pip
pip3 install rospkg dataclasses scipy numpy pyqtgraph
```

5. Further useful setup:
add at the end of your .bashrc file:
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.sh
```
To run ROS across multiple machines, also add:
```bash
export ROS_IP=*your local IP*
export ROS_MASTER_URI=http://*your local IP*:11311
```

Use an environment loader to do the exports on the local machine (see onboard_interface for examples)

