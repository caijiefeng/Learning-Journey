# **RmHw**
## **README**
rm\_hw是ROS中机器人硬件与RoboMaster电机的控制接口

*用法*
***roslaunch rm_hw rm_hw.launch***
or better use ***mon launch rm_hw rm_hw.launch***

*节点*
rm\_hw

## **h**
### **HardwareInterface(folder)**
#### **types.h**
*ActCoeff结构体*
在rm\_hw的命名空间下定义的结构体，其中包含了Cheetah电机的一些相关系数
*ActData结构体*
包含RoboMaster电机编码器需要的一些数据
*ImuData结构体*
包含imu中的一些数据
*CanDataPtr结构体*
将ActData、ActCoeff、ImuData中数据转换为can总线可识别的代码

#### **socketcan.h**
这个库中的所有数据都被定义在了can的命名空间下
***can命名空间***
*SocketCan类*
其中封装了open、close、is\_open、write、start\_receiver\_thread等函数以及can总线需要收发的一些数据和请求

#### **CanBus.h**
*CanFrameStamp结构体*
执行了can\_frame以及ros::time的命令
*CanBus类*
声明了CanBus以及write、read、frameCallback等一系列函数，并且封装了一些与can有关的数据(变量)

#### **hardwareinterface.h**
这个库所有的内容都是命名在*rm_hw*的命名空间下
*RMRobotHW类*
以public的方式继承了hardware\_interface::RobotHW类，对RoBotHW中的相关函数进行了重写，并且封装了一系列函数以及数据(变量)

## **cpp**
### **HardwareInterface(folder)**
#### **socketcan.cpp**
*open函数*

