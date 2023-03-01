# rm_bringup

rm_bringup是rm_controls中一个重要的部分，它用于实现机器人的自启，使得机器人在接通电源后可以立刻开始对其进行控制，就不用再使用电脑连接nuc/upboard后再打开相关的程序。

通常来说rm_bringup主要做三件事：

* 自动set can
* 自动打开roscore并配置好相关环境变量
* 自动运行机器人所需的所有程序

rm_bringup本质上是通过shell脚本和linux下的service脚本实现的。

---

## xxx.sh

这个脚本负责执行相应的机器人自启程序，例如：set can、配置环境变量、配置本机ip、打开roscore、执行机器人控制程序等。

**dynamicx**示例如下：

```shell
#!/bin/bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=~/rm_ws:$ROS_PACKAGE_PATH
source ~/rm_ws/devel/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
export ROBOT_TYPE=hero
export CAMERA_TYPE=mv_camera
export ENEMY_COLOR=red
export ROS_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep -v '172.17.0.1')
if test -z "${ROS_IP}"; then
  export ROS_IP=$(dig +short localhost)
fi
mon launch --disable-ui rm_bringup start.launch
```

---

## xxx.service

这个脚本负责完成自启工作，这也是机器人为什么能够自动执行xxx.sh的原因所在。linux新的发行版内置了systemctl来执行service脚本。

首先学习service脚本：

1. systemd有系统和用户的分别：

   通常**系统目录（/usr/lib/systemd/system/）**而**用户目录（/etc/lib/systemd/user/）**，一般情况下我们的service脚本会存放再系统目录下。

2. [Unit]：

   这个字段给出service描述**（Description）**、启动顺序**（After/Before）**以及依赖关系**（Requires）**。

   [Service]：

   这个字段给出service启动类型**（Type）**、启动行为**（Execxxx）**。

   [Install]：

   这个字段定义如何做到开机自启**( WantedBy)**。

**dynamicx**示例如下：

```shell
[Unit]
Description=rm auto startup
After=network-online.target
[Service]
User=dynamicx
Type=simple
#Type=forking
ExecStart=/bin/bash -c "~/rm_start.sh"
[Install]
WantedBy=multi-user.target
#定义了该service所在的target，Target的含义是服务组，表示一组服务。WantedBy=multi-user.target指的是服务所在的Target是multi-user.target。Systemd有默认的启动Target。就是multi-user.target，在这个组里的所有服务，都将开机启动。 
```

---

## create_xxx.sh

这个脚本负责将写好的xxx.sh服务以及xxx.service放到合适的路径下。

**dynamicx**示例如下：

```shell
#!/bin/bash
echo " "
echo "Start to copy script files to /home/dynamicx"
echo ""
cp `rospack find rm_bringup`/scripts/auto_start/start_master.sh ~/
cp `rospack find rm_bringup`/scripts/auto_start/rm_start.sh ~/
chmod 777 ~/start_master.sh
chmod 777 ~/rm_start.sh
echo " "
echo "Start to copy service files to /lib/systemd/system/"
echo ""
sudo cp `rospack find rm_bringup`/scripts/auto_start/start_master.service  /lib/systemd/system/
sudo cp `rospack find rm_bringup`/scripts/auto_start/rm_start.service  /lib/systemd/system/
echo " "
echo "Enable auto rm start! "
echo ""
sudo systemctl enable start_master.service
sudo systemctl enable rm_start.service
echo "Finish "
```

---

## udev

rm_bringup下的udev负责完成物理映射，包含Dbus和refreee端口到usb的映射。

具体步骤如下：

* pc端连接usbtocan(dbus)

* ```shell
  ls /sys/class/tty/ttyUSB* -l
  #其中"1-2-1:1.0"为usbDbus的物理地址
  ```

* ```shell
  cd /home/dynamicx/rm_ws/src/rm_bringup/srripts/udev
  vim rm.rules
  #修改KERNELS=="${usb物理地址}"
  ```

* ```shell
  ls /dev/ | grep usb
  #重启后查看是否有usbDbus和usbRefree，有则映射成功
  ```

* ```shell
  rosrun rm_dbus rm_dbus
  #检测是否可以启动rm_dbus的node
  ```
