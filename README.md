# Installation
*These steps have been tested on Ubuntu 20.04, making ROS Noetic work on newer versions of ubuntu might be possible but is a little bit trickier*

After installing Ubuntu 20.04:
1. Install ROS Noetic ([Official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu))
2. Install Eigen:
Download eigen version >3.4 [here](https://gitlab.com/libeigen/eigen), open the folder in a terminal
```bash
sudo cp -r -t /usr/local/include/ Eigen/ unsupported/
```
3. Install Boost:
```bash
sudo apt install libboost-dev
```
4. Install python dependencies:
```bash
sudo apt install python3-pip
pip3 install rospkg dataclasses scipy numpy pyqtgraph pyserial
```

5. If you have never installed another ROS package before:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

6. Install Simulator package and GUI plugin:
Clone or download repository in your catkin workspace (usually ~/catkin_ws/src if you followed the ROS tutorials), then compile it:
```bash
cd ~/catkin_ws/src
git clone https://github.com/EPFLRocketTeam/real_time_simulator.git
git clone -b noetic-devel https://github.com/OTL/rqt_ez_publisher.git
cd ~/catkin_ws
catkin_make
source ~/.bashrc
rqt --force-discover # Then close windows
```


Rviz configuration: The configuration of the rviz plugin in rqt will not load properly because it only stores the absolute path in the .perspective file.
This is fixed by writing your absolute path in GUI/rocket_GUI.perspective, then stop tracking the file to keep your personal change to the GUI local
```bash
cd ~/catkin_ws/src/real_time_simulator
sed -i 's/username/'"$USER"'/g' GUI/rocket_GUI.perspective
git update-index --assume-unchanged GUI/rocket_GUI.perspective

```

# Test
Once everything is installed, you can use the test_simu.sh script to simulate a basic flight and check that everything is properly working
```bash
roscd real_time_simulator/
./bash_scripts/test_simu.sh # Then ctrl+c inside terminal to stop simulation
```
