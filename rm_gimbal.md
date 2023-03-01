# RmGimbal
rm\_gimbal\_controllers 具有三种状态:RATE、TRACK 和 DIREC，其中在RATE模式下可以根据遥控器的信号移动，进入TRACK模式后云台会自动追踪目标并移动到追踪位置，在DIREC模式下云台可以移动到设定的点位。

云台控制器可以根据指令对偏航joint和俯仰joint进行PID控制,还能根据检测到的数据进行平均滤波,并且根据弹道模型计算、预测和跟踪目标。
[移动平均滤波器物](https://baike.baidu.com/item/%E7%A7%BB%E5%8A%A8%E5%B9%B3%E5%9D%87%E6%BB%A4%E6%B3%A2%E5%99%A8/3901084?fr=aladdin)
[滤波器](https://baike.baidu.com/item/%E6%BB%A4%E6%B3%A2%E5%99%A8/2551370?fr=aladdin)

## 使用的硬件接口类型
*JointStateInterface*
用于获取云台joint的速度和位置
*EffortJointInterface*
向云台joint上发送扭矩命令
*RoboStateInterface*
用于获取(1)云台和世界坐标系之间的当前和历史转换以及(2)视觉目标和世界坐标系之间的转换

## 使用用法
***mon launch rm_config load_controllers.launch***

## **Launch文件类型**
*load_controllers.launch*
可以加载出配置文件中的参数gimbal\_controller

## ROS的API接口
### 订阅话题
*command (rm_msgs/GimbalCmd)*
设置云台模式、俯仰和偏航轴旋转速度、跟踪目标、指向目标和坐标系
*/detection (rm_msgs/TargetDetectionArray)*
接收视觉识别上的数字
*/<camera_name>/camera_info (CameraInfo)*
确保检测节点能够接收到新的图像帧并且向检测节点发送预测的数据

### 发布话题
*error (rm_msgs/GimbalDesError)*
通过弹道模型计算出当前云台的角度对目标射击的误差
*track (rm_msgs/TrackDataArray)*
一种给检测节点用于确定ROI的预测数据

*model_desire(visualization_msgs/Marker)*
用于子弹轨迹的可视化
*model_real(visualization_msgs/Marker)*
用于通过当前云台角度下弹道模型计算出的轨迹可视化

### 相关参数
这些参数都存储在rm_gimbal_controller下的config文件夹中的yaml参数配置文件内
*detection_topic*
检测节点获取数据的topic名称
*detection_frame*
检测帧的名称
*camera_topic*
确定检测节点接收到一帧新图像并将预测数据发送给检测节点的topic名称
*publish_rate*
发布消息的频率
*chassis_angular_data_num*
底盘角度数据的个数（用于底盘角度的平均滤波）
*time_compensation*
图像传输的延迟时间（用于减小这种影响）
*`bullet solver `*
//用于获取子弹落点的相关参数
*resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18, resistance_coff_qd_30*
qd后面的数字为子弹数字,表示在特定的子弹速度时的空气阻力系数
*g*
重力加速度
*delay*
shooter在接收到发射指令之后的发射延迟时间,用于减少延迟时间带来的影响
*timeout*
*bullet solver*超出的时间,用来判断*bullet solver*是否能成功计算出子弹的落点
*` Moving average filter `*
//移动平均滤波器用于过滤掉旋转目标的装甲中心
*is_debug(bool)*
把这个选项调为true时,将会在filter的topic上发布调试的数据
*pos_data_num*
装甲位置数据的数量
*vel_data_num*
装甲速度数据的数量
*gyro_data_num*
目标转速数据个数
*center_data_num*
目标旋转中心数据个数
*center_offset_z*
z轴上的偏移量,用于减小z轴滤波误差带来的影响

## example
```C++
gimbal_controller:
    type: rm_gimbal_controllers/Controller
    time_compensation: 0.03
    publish_rate: 100
    chassis_angular_data_num: 20
    camera_topic: "/galaxy_camera/camera_info"
    yaw:
      joint: "yaw_joint"
      pid: { p: 8, i: 0, d: 0.4, i_clamp_max: 0.0, i_clamp_min: -0.0, antiwindup: true, publish_state: true }
    pitch:
      joint: "pitch_joint"
      pid: { p: 10, i: 50, d: 0.3, i_clamp_max: 0.4, i_clamp_min: -0.4, antiwindup: true, publish_state: true }
    bullet_solver:
      resistance_coff_qd_10: 0.45
      resistance_coff_qd_15: 0.1
      resistance_coff_qd_16: 0.7
      resistance_coff_qd_18: 0.55
      resistance_coff_qd_30: 3.0
      g: 9.81
      delay: 0.1
      dt: 0.001
      timeout: 0.001
      publish_rate: 50
    moving_average_filter:
      is_debug: true
      center_offset_z: 0.05
      pos_data_num: 20
      vel_data_num: 30
      center_data_num: 50
      gyro_data_num: 100
```

---

## bullet_solver

**include：**

* <realtime_tools>：实时工具，用于确保数据的实时性
* <geometry_msgs/TransformStamped.h>：ros官方提供的用于存储tf的变量

**config结构体：**
config中定义了多种子弹速度下的空气阻力系数、重力加速度、延时、dt、时间误差，分别与config文件下的yaml文件对应
**封装的函数：**

* getGimbalError函数                获取云台误差

* getResistanceCoefficient函数      获取空气阻力系数

* solve函数                         解算子弹落点

* bulletModelPub函数                发布弹道轨迹模型

* reconfigCB函数                    重新初始化config中存储的参数



#### *getResistanceCoefficient函数*

这个函数用来获取空气阻力系数
1.当子弹速度小于12.5m/s时,这个系数统一使用*qd_10*参数所设定的值
2.当子弹速度小于15.5m/s时,这个系数统一使用*qd_15*参数所设定的值
3.当子弹速度小于17m/s时,这个系数统一使用*qd_16*参数所设定的值
4.当子弹速度小于24m/s时,这个系数统一使用*qd_18*参数所设定的值
5.其它的子弹速度,系数设置为*qd_30*所设定的值
这些参数可以在yaml参数配置文件里查找,每一个机器人不完全相同

#### *solve函数*

getResistanceCofficient函数中获取的系数要进行一次判断,如果不为0就取,为0就设置为0.001
（C++知识补充）
std::atan2(y,x)函数用于获取y/x的反正切,即y=arctanx
std::pow(x,y)用于求解x的y次幂,即x^y
std::sqrt(x)用于求解x的平方根
std::cos(x)用于求解x的余弦
std::sin(x)用于求解x的正弦
std::exp(x)用于求解e^x
std::isnan(x)用于判定x是否为非数字（NaN）,是返回0,不是返回1
std::log(x)用于求解log(e)(x),即以e为底的x的对数
```C++
output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));double fly_time =
    (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) * (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -config_.g * fly_time / resistance_coff_;
```
利用目标与Output的位置（x,y,z）三个坐标轴上的差值经过一系列公式循环计算进而得出最精确的*error*,直到*error*小于0.001,如果*error*为非数字或者循环次数等于或大于20次退出循环并且宣布solve失败
[yaw,pitch,roll三轴旋转](https://blog.csdn.net/qq_38800089/article/details/108768388)
[yaw,pitch,roll三轴坐标系计算](https://blog.csdn.net/jxyyl/article/details/119561637)

#### *bulletModelPub函数*

解算出point容器下的desire和real位置，再存储到point容器下，再根据此时point容器下存储的desire和real数据给子弹轨道的desire和real赋值，完成弹道轨迹模型发布的完整功能
注意这其中有一个四元素转换为rpy坐标系的过程

#### *getGimbalError函数*

这个函数用于获取云台控制器的误差
（C++知识补充）
std::abs(x)返回x的绝对值
利用目标与real（x,y,z）三个坐标轴上的的位置经过一系列计算公式循环计算出*fly_time*,直到两次*fly_time*小于0.01,当循环次数超过或者等于20次的时候,直接返回999
如果两次*fly_time*通过循环成功降到0.01以下,再通过特定计算公式得到real\_Z以及目标的yaw进而算出误差(error)并且返回

#### *reconfigCB函数*

reconfigCB函数用于重新对使用过的参数初始化为yaml参数配置文件中的数值

---

## gimbal_base

云台控制器主要拆分为两个类，一个bullet_solver负责枪管角度解算，gimbal_base则负责云台的基本功能包含云台位置纠正，云台的移动等等

### include：

* <effort_controllers/joint_position_controller.h>：ros_control提供的位置控制器类型，获取位置数据转换为力矩输出
* <hardware_interface/imu_sensor_interface.h>：获取imu数据的接口
* <realtime_tools/realtime_publishr.h>：实时发布器保证数据的实时性
* <rm_msgs/GimbalCmd.h>：自定义gimbalcommand消息类型用于向云台发布命令，这里可以获取命令消息类型
* <rm_msgs/TrackData.h>：自定义自瞄相关消息类型
* <rm_msgs/GimbalDesError.h>：自定义云台预期位置错误消息类型

### 封装函数：

***init函数***

控制器初始化，这里完成了需要的成员变量检测，并从参数服务器中获取需要的参数赋给定义好的变量，同时各个订阅者开始订阅所需的话题数据，并配置好tf转换

***starting函数***

控制器打开时执行，将云台模式设为RATE

***update函数***

控制器打开期间不断重复循环执行，不停地从实时buffer中读取GimbalCommand数据以及track数据，根据读取到的数据移动云台

***setDes函数***

设定云台预期位姿的tf

***rate函数***

云台处于rate模式下执行（在update函数中调用），根据setDes函数设定的云台预期位姿调整云台实时位姿（赋予云台gimbalDes的transform）

***track函数***

云台处于track模式下执行，根据track数据追踪目标

***direct函数***

云台处于direct模式下执行，将设定点的transform传给云台

***setDesIntoLimit函数***

将云台预期位姿变动范围在urdf中设定的软限位之内

***moveJoint函数***

顾名思义移动关节，根据云台获取到的tramsform将云台的pitch、yaw关节移动到指定位置
