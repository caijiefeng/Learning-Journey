# VIO
## 简介
VIO全称Visual Inertial Odometry（视觉惯性里程计），是一种计算机视觉技术，用于估算移动车辆（无人机）相对于本地起始位置3D位置（局部的位置和方向）以及速度。

VIO通常用于没有GPS或者GPS方案不可行/不可靠的情况下（室内、桥下）导航车辆（无人机）。
VIO使用Visual Odometry（视觉历程计）来从摄像头的图像中估计车辆（无人机）姿态，同时结合机载IMU的惯性测量值来纠正车辆（无人机）快速移动时的错误从而导致的图像捕捉不佳，也可以说VIO是一个用于坐标转换并发送信息的节点。

这里对于VIO的介绍更多是关于PX4官方提供的ROS软件包，而不是视觉惯性里程计，关于视觉惯性里程计的相关内容可以看另一篇笔记[Visual-SLAM](control/drone_control/my_drone_control/px4_d435_and_t265/Visual-SLAM.md)。

PX4并不关心消息的来源，只要是合适的MAVLink接口提供的消息PX4都会接收，根据这一特性，我们可以通过ROS来把VIO的信息流到PX4上。

### VIO支持的设置
VIO支持使用T265相机和使用ROS的机载电脑为PX4提供里程信息，VIO bridge ROS node这个节点为相机和ROS之间通信提供了桥梁。
### 相机安装
将相机和机载计算机连接并把相机安装到机架上，需要注意的是：
* 使用所提供的电缆连接T265相机
* 尽可能把镜头向下安装相机
* 使用防振泡沫安装相机（相机对振动非常敏感）
### ROS/VIO设置
* 在机载电脑上安装并配置好MAVROS
* 获取VIO bridge ROS node:
```shell
cd ~/catkin_ws/src
git clone https://github.com/Auterion/VIO.git
#将VIO包拉到本地的工作空间中
cd ~/catkin_ws/src
catkin build px4_realsense_bridge
#编译VIO包
```
需要注意的是，在你编译px4_realsense_bridge之前，需要先安装好[librealsense](https://github.com/IntelRealSense/librealsense)。
```shell
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
#注册服务器公钥
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
#将服务器添加到存储库列表中
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
#后面两个可选择
sudo apt-get update
sudo apt-get upgrade
#完成安装
```
* 配置相机方向
如果安装时相机向下安装的，则VIO bridge不需要任何配置，如果安装方向不是向下，其它方向修改在```bridge_mavros.launch```中：
```xml
<node pkg="tf" type="static_transform_publisher" name="tf_baseLink_cameraPose"
    args="0 0 0 0 1.5708 0 base_link camera_pose_frame 1000"/>
<!-- 这个是一个坐标系的静态转换，类似于urdf里的joint中子link到父link的偏移量，这个参数设定决定了camera_pose_frame到无人机的base_link的相对位置，前面三个参数分别对应xyz，后面的对应yaw、pitch、roll -->
```
* 配置参数

  VIO在启动时通常我们会希望它能一起启动mavros，因为这样可以为我们的操作带来便利，需要在```bridge_mavros.launch```中配置以下参数：

```xml
<arg name="fcu_url" default="/dev/ttyPixhawk:921600"/>
<!-- fcu_url设置为连接在电脑上的Pixhawk设备的端口，这是为mavros与PX4通信搭建的桥梁 -->

<arg name="gcs_url" default="" />   <!-- GCS link is provided by SITL -->
<!-- 如果你不希望通过有线连接的方式来让QGC连接飞控，可以设置gcs_url这个参数来让QGC通过mavros连接到飞控，gcs_url跟着的是QGC主机的ip -->
```

```shell
#示例：
roslaunch px4_realsense_bridge bridge_mavros.launch gcs_url:=udp://@localhostip

#or:
roslaunch px4_realsense_bridge bridge_mavros.launch gcs_url:=udp-b://@
```

* 运行VIO

```shell
roslaunch px4_realsense_bridge bridge_mavros.launch
#在大多数情况下使用，启动VIO与ROS通信的bridge以及MAVROS
roslaunch px4_realsense_bridge bridge.launch
#MAVROS在其它组件中启动情况下使用
roslaunch px4_realsense_bridge bridge_mavros_sitl.launch
#用于模拟
```
* 检测是否与飞控连接
使用QGC上带有的MAVLink Inspector来查看是否接收到了来自```ODOMETRY ```或者 ```VISION_POSITION_ESTIMATE```的消息，也可以或检查组件 id 为 197 (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY) 的 HEARTBEAT 消息。
* 在首次飞行时一定要确保VIO的设置正确

### PX4 Tuning

要想使得VIO可以访问EKF2的外部数据，需要在QGC中调整VIO的相关参数，具体需要调整的参数可以在PX4.md中看到。

### 检查VIO

在首次飞行时，执行下面的检查VIO是否正常工作：

* 在QGC中将PX4参数```MAV_ODOM_LP```设置为1，此时PX4应该能够接收到外部位姿作为MAVLink ```ODOMETRY``` 的消息返回，这些消息可以在QGC中的```MAVLIink Inspector```中看到。
* 使车辆偏航，直到```ODOMETRY```中的消息的四元数接近单位四元数（w=1,x=y=z=0）。此时的车身框架应该已经与外部姿态系统的参考系框架对齐，在这一步对齐坐标系时，如果你对车辆进行了俯仰、翻滚的操作才对齐的坐标系，请不要飞行，重新检查并对齐坐标系。车辆对齐后，拿起车辆z轴坐标会减小，向前移动x轴坐标增大，向右移动y轴坐标增大。
* 检查线速度的消息是否出现在FRD参考坐标系中。
* 完成上面3步后，将PX4中的参数```MAV_ODOM_LP```设置为0，PX4会停止返回```ODOMETRY```的消息。

