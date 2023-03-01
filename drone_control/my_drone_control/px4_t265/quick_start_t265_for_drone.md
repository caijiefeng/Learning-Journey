# The quick_start of t265

Before you read this tutorial, **you must first read [t265_refer_links](https://github.com/N-XIX-L/RM-Drone-Tutorial/blob/master/2022/px4_t265/t265_refer_links.md) or follow the online tutorials to ensure a certain understanding of drones,** *including but not limited to being able to assemble and disassemble small drones by yourself, and to be able to wire drones by yourself, Familiar with the basic principles, parameters and functions of [PX4 autopilot](https://docs.px4.io/main/en/), understand the communication method of drone, understand [VIO](https://github.com/Auterion/VIO), [t265](https://www.intelrealsense.com/tracking-camera-t265/), [mavros](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) and be proficient in using the common functions of [QGC](https://docs.qgroundcontrol.com/master/en/index.html).*

---

## The logic of the whole process

Firstly,we should understand why we need to use the t265 on our drone.Because sometimes we need to use drones in places such as indoors or under bridges, but GPS is often unreliable or even unavailable in these places. If you still rely on GPS to provide location information at this time, it cannot meet the requirements for take-off. , so we choose to use t265 instead of GPS to provide us with location information.

After understanding why we need to use t265, we also need to understand how t265 provides the location information it has collected to PX4.

*After startup,t265 will automatically obtain real-time pose information, and realsense_ros will publish the pose information on the ros topic, and mavros will subscribe to the messages on these topics and convert them into MAVLink messages that PX4 can receive, and PX4 will respond to these MAVLink messages. The message is processed and sent to various devices connected to the flight controller to ensure the stable flight of the entire drone. Similarly, the messages fed back by various devices and sensors are finally published on the ros topic through the same process.*

The above is the logic of the entire t265 process operation.This also explains why the flight control is called the core of the whole drone.

---

## Run the whole process on your computer

**You must run the entire procedure on your computer to make sure there are no errors before attempting to fly indoors.**

By following the steps below you can quickly determine if you have what it takes to try flying indoors for the first time:

* Make sure you have already installed [QGC](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html) on your computer.

```shell
#Before installing QGC.
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
```

* Then install the intel camera driver.

```shell
#Install the librealsense.
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

#Test if you have completed the installation.
realsense-viewer
```

* Install the realsense-ros.

```shell
#Some dependent packages must be installed.
sudo apt-get install ros-noetic-ddynamic-reconfigure
#Install realsense-ros from source.
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
catkin clean
catkin build
```

* Install the MAVROS.

```shell
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
#This step of the installation will take some time and be patient.
```

* Install VIO.

```shell
#Some dependent packages must be installed.
sudo apt install ros-noetic-pcl-ros
#Install realsense_px4_bridge from source.
cd rm_ws/src
git clone https://github.com/Auterion/VIO.git
cd ..
catkin build
#If that fails try catkin clean.
catkin clean
catkin build
```

* Check.

```shell
roscore
#Use rviz to check if the camera is working.
roslaunch realsense2_camera demo_t265.launch
#Check if the computer communicates with the flight controller.
roslaunch mavros px4.launch fcu_url:=//dev/ttyUSB3:921600
```

