# ros control
## **overview**
roscontrol是一种对pr2包的重写,使得控制器对pr2以外的所有机器人通用

ros control从编码器上获取机器人的状态和数据作为输入,并且通过控制器的反馈机制（PID控制器）来调控执行器的输出（力矩）

大体上roscontrol可以看成一个三级关系之间的转换传输,*controller、hardwareinterface、actuator*三者之间的调控,*hardwareinterface*从机器人的编码器上获取相关的数据作为输入,再在*controller*中通过一定的转换关系将这些数据转换为所需要的输出,并再次通过*hardwareinterface*输出到执行器上,*controller*和*hardwareinterface*之间可以抽象出一个*joint*的关系层,这个*joint*就是可以理解为我们常说的电机

也就是说ros control可以说把ros对于机器人的控制转换为了对*joint*的控制,而对*joint*的控制又分成了三层的关系,*controller*是最上层的

##**controller**
roscontrol中自带了一些可用的控制器插件（也可以自己写）

###*joint_state_controller*
将JointStateInterface上的数据发布到sensor\_msgs/JointState的topic上

###*position_controllers*
向hardwareinterface上命令一个需要的位置
####*joint_position_controller*
接收一个位置输入并且输出一个位置输出,使用*foward_command_controller*来传输输入
####*joint_group_position_controller*
一次传输、设置多个位置

###*velocity_controllers*
向hardwareinterface上命令一个速度
####*joint_position_controller*
使用pid控制器接收一个位置并输出一个速度
####*joint_velocity_controller*
接收一个速度输入并且输出一个速度输出,使用*foward_command_controller*来传输输入
####*joint_group_velocity_controller*
一次传输、设置多个速度

###*effort_controllers*
向hardwareinterface上命令所需要的力或者力矩
####*joint_position_controller*
接收一个位置输入并输出一个力矩输出（使用PID控制器）
####*joint_group_position_controller*
一次传输、设置多个位置
####*joint_velocity_controller*
接收一个速度输入并输出一个力矩输出（使用PID控制器）
####*joint_effoet_controller*
接收一个力矩输入并输出一个力矩的输出,使用*foward_command_controller*来传输输入

###*joint_trajectory_controllers*
一些额外的插件功能

##**Hardware Interfaces**
roscontrol中自带了一些硬件接口,硬件接口与controller一起向robot（执行器）发送命令（也可以自己写）

###*Joint Command Interface*
可用于设置任何一类命令,只要这些命令可以使用单个双精度来表示
####*Effort Joint Interface*
命令基于力矩的joint
####*Velocity Joint Interface*
命令基于速度的joint
####*Position Joint Interface*
命令基于位置的joint

###*Joint State Interfaces*
用于读取一组已经命名好的joint,这个joint中包含位置、速度和力矩

###*Actuator State Interfaces*
用于读取一组已经命名的actuator,这个actuator中包含位置、速度和力矩

##**Transmissions**
Transmission用于传输输入,使得输入的一些变量到输出的过程中保持功率不变
机械传动的公式:**P=FxV,P1=P2,F1xV1=F2xV2**
比例为n的机械减速器:**Fjoint=nFactuator,Vjoint=Vactuator/n**

###*URDF Transmission*
[URDF Transmissions](https://wiki.ros.org/urdf/XML/Transmission)

##**Joint Limits**
Joint\_limits\_interface中有对joint限制以及joint的相关命令限制的相关数据以及方法（可以在URDF和rosparam中填充）
controller不会使用joint\_limits\_interface,而是在controller更新完毕之后再执行限制操作（防止限制命令覆盖了controller设置的命令）

**规格:**
1.joint限制位置、速度、加速度、加加速度和力矩
2.软限位（k\_p,k\_v）
3.在URDF中写入joint限制的位置、速度、力矩的方法
4.在URDF中写入软限位的方法
5.在ros参数服务器中写入joint限制的方法





