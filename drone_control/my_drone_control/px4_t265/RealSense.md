# RealSense

RealSense技术为我们带来了更加沉浸直观的体验，又同时搭载了Intel强大的处理器，为我们重新定义了人机交互sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE In case the public key still cannot be retrieved, check and specify proxy settings: export http_proxy="http://<proxy>:<port>"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
realsense-viewer方式，这项技术让计算机理解人的动作、情感成为可能。

## [librealsense](https://github.com/IntelRealSense/librealsense)

英特尔® 实感™ SDK 2.0 是一个跨平台库，适用于英特尔® 实感™ 深度摄像头（D400 和 L500 系列以及 SR300）和 T265 跟踪摄像头。简单来说，它是一个开源的相机驱动，但它不包含计算机视觉算法。

运行以下命令，在你的ubuntu平台下安装librealsense库并开始使用：

```shell
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE In case the public key still cannot be retrieved, check and specify proxy settings: export http_proxy="http://<proxy>:<port>"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
realsense-viewer
```

---

## [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

realsense-ros是一个将英特尔实感摄像头（D400 系列 SR300 摄像头和 T265 跟踪模块）与 ROS 配合使用的软件包。这个软件包支持Kinetic、Melodic、Noetic版本的ROS，在ROS2下运行需要切换到ros2分支。

realsense-ros的作用实际上可以理解为一个读取数据并且发布topic的节点，它会驱动相应的相机，读取相机的数据同时把这些数据发布到ROS的topic上，在编写程序时候去订阅这个“节点”发布的topic即可。

在安装realsense-ros之前，确保ddynamic_reconfigure已经被安装在你的电脑里，如果没有运行以下命令：

```shell
sudo apt-get install ros-noetic-ddynamic-reconfigure
```

运行以下命令，在你的ubuntu平台上安装realsense-ros：

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```