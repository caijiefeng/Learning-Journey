# MAVROS
MAVROS上是一个ROS的软件包，它可以为运行ROS的计算机提供与使用MAVLink的自动驾驶仪、地面控制站以及其它设备通信的通道。通俗地讲，MAVROS可以把ROS话题转化为MAVLink格式的消息发送给飞控，飞控中负责处理MAVLink消息的模块又将MAVLink格式的消息转化为uORB消息给PX4各个模块使用，反过来，飞控各模块反馈给PX4的uOBR消息经飞控里的MAVLink模块处理后转化为MAVLink格式消息发给计算机，计算机上的MAVROS处理后转化为ROS话题。
MAVROS允许和任何使用MAVLink的自动驾驶仪通信，但我们主需要关注如何在PX4自动驾驶仪与运行ROS的计算机之间进行建立通信通道。
在Ubuntu Linux上带有ROS的Gazebo中可以找到一个标准安装脚本，会自动帮我们安装PX4、ROS、Gazebo仿真环境、MAVROS，但是对于我们来说ROS、PX4、Gazebo都已经安装完成，我们只需要关注如何安装MAVROS。
## MAVROS安装
[官方安装文档](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
官方提供了二进制安装以及从源安装两个方案。
运行以下命令安装ROS Noetic版本MAVROS:
```shell
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
#使用apt-get安装ROS Noetic版本MAVROS
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
#使用install_geographiclib_datasets.sh脚本安装GeographicLib库，这需要一定时间，耐心等待即可
```
## 节点
### mavros_node
MAVROS中最主要的通信节点。
* 订阅话题：
-mavlink/to (mavros_msgs/Mavlink) Mavlink流到自动驾驶仪
* 发布话题：
-mavlink/from (mavros_msgs/Mavlink) 来自自动驾驶仪的Mavlink流
-diagnostics (diagnostic_msgs/DiagnosticStatus) 诊断状态的消息
* 重要参数：
-~system_id (int, default: 1) MAVLink节点系统ID
-~component_id (int, default: 240) MAVLink节点组件ID
-~target_system_id (int, default: 1) MAVLinkFCU系统ID
-~target_component_id (int, default: 1) MAVLink组件系统ID
-~startup_px4_usb_quirk (bool, default: false) 是否为PX4定义
-~plugin_blacklist (string[], default: []) 别名黑名单
-~plugin_whitelist (string[], default: []) 别名白名单
-~fcu_url (string, default: /dev/ttyACM0:57600) FCU连接URL
-~fcu_protocol (string, default: v2.0) MAVLink协议版本，支持“v1.0"、"v2.0"以及"0.18“中的新功能
-~gcs_url (string, default: udp://@) GCS网桥连接URL
### gcs_bridge
额外的代理，以前的名字叫ros_udp。
* 订阅话题：
-mavlink/from (mavros_msgs/Mavlink) 来自自动驾驶仪的MAVLink流
* 发布话题：
-mavlink/to (mavros_msgs/Mavlink) MAVLink流到自动驾驶仪
* 参数：
-~gcs_url (string, default: udp://@) 连接URL
### event_launcher
监听arming状态，触发并执行事件程序。
* 订阅话题：
-mavros/state (mavros_msgs/State) arming事件的来源
* 服务：
-trigger_event (std_srvs/Trigger) 通过服务创建由event_name参数定义的事件
* 参数：
-~<event_name>/service (string) 创建由trigger_event服务触发的事件
-~<event_handler>/event (string[]) 触发该处理程序的事件列表
-~<event_handler>/action (string[]) 对相应事件执行的actions列表，支持的actions：run、stop
-~<event_handler>/shell (string) 对相应事件执行run、stop的命令
-~<event_handler>/logfile (string, default: "") 用于保存命令的 stdout 和 stderr 输出的文件

## 用法
MAVROS的launch文件位于mavros/launch目录下，运行以下命令以开启从ROS话题到MAVLink的通信通道：
```shell
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://@172.16.254.1
#这是一个例子，通过修改MAVROS包中的默认参数来使MAVROS将飞控与计算机连接。fcu_url指定的是飞控所连接的端口#设置正确的飞控连接端口即可。gcs_url指定计算机的ip。
#通常下面这一行命令会更加常用，因为它可以省去输入主机ip
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp-b://@
#FCU是一个5G智能网关。URL是统一资源定位符，是对可以从互联网上得到的资源的位置和访问方法的一种简洁的表示。
#
```

## 实用命令
所有的这些实用命令都可以加上--help的参数来获取帮助，这些命令都可以通过QGC地面控制站发送给飞控，这些命令在平常用到的并不多，仅作为在飞控报错时更快定位错误的工具。

打开QGC，点击左上角的“Q”图标>点击“Analyze Tools”>选择"MAVLink Console"。

### mavcmd
用于向MAVLink设备发送命令的命令行工具。
```shell
#命令使用格式
mavcmd [-h] [-n MAVROS_NS] [-v] [--wait]
              {long,int,sethome,takeoff,land,takeoffcur,landcur,trigger_control}
              ...
#具体命令功能              
mavcmd long / mavros int
#发送任何命令
mavcmd sethome
#请求改变初始位置
mavcmd takeoff
#请求起飞
mavcmd land
#请求在当前位置着陆
mavcmd takeoffcur
#请求从当前GPS坐标起飞
mavcmd landcur
#请求在当前GPS坐标着陆
mavcmd trigger_control
#控制板载摄像头触发系统（PX4）
```
同时mavcmd工具还提供了许多参数：
* -h / --help
获取帮助信息并退出。
* -n MAVROS_NS / --mavros-ns MAVROS_NS
ROS节点的命名空间。
* -v / --verbose
 显示详细输出。
* --wait
 等待建立FCU连接。
### mavftp
仅适用于与PX4建立通信，是一个MAVLink-FTP的文件操作工具。
```shell
#命令使用格式
mavftp [-h] [-n MAVROS_NS] [-v]
              {cd,list,cat,remove,mkdir,rmdir,download,upload,verify,reset}
              ...
#具体命令功能
mavftp cd 
#切换目录，与cd功能一致
mavftp list
#列出文件和目录
mavftp cat
#查看文件,与cat功能一致
mavftp remove
#删除文件，与remove功能一致
mavftp mkdir
#创建目录，与mkdir功能一致
mavftp rmdir
#删除目录，与rmdir功能一致
mavftp download
#下载文件
mavftp upload
#上传文件
mavftp verify
#验证文件
mavftp reset
#重置
```
***MAVROS更多详细命令可以在mavros官方wiki中找到***
              

