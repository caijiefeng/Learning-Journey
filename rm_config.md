# rm_config
config(配置)是我们上实车调试必须设置好的配置，配置没有写好则无法匹配相应的电机执行相应的命令
如果在仿真中能正常实现，而在实车上无法正常运行，那么只可能是配置出现了问题 

目前已经使用的launch有两个rm_hw.launch、load_controllers.launch，我们控制实车的方式其实是通过控制运动关节joint来对我们的link做相应的控制，也就是urdf中所写的joint、link，joint可以等效地看作电机，而我们又可以通过控制电调进一步控制电机，注意不是所有的电机都配有电调
## rm_hw
### launch
rm_hw.launch负责将实车上的各个joint所配对的电机与我们所建立的通信接口连接起来，它会读取rm_config包下的rm_hw库中所设立的机器人类型的配置文件
### config
rm_hw中的config文件需要注意的点有:
id     通用的格式都是0x___，imu为0x10_,其他电机为0x20_
bus    分为can0以及can1，can0和can1根据接口决定
type   电机的类型3508、2006、6020根据实车上装配的电机决定
对应电机的例子:
```yaml
actuators:
  upper_left_friction_wheel_joint_motor:
    bus: can0
    id: 0x201
    type: rm_3508
    lp_cutoff_frequency: 60
    lp_debug: true
```
我们有时不需要使用整车进行测试，这时只需要我们将需要测试的joint的id、bus、type这些设置对就行，注意不需要的can口要删除或者注释掉，rm_hw中只加载相应的电机，但不允许can口未初始化 
## rm_controllers
### launch
load_controllers.launch负责加载各种参数服务器进入控制器，不同于各种controllers包下的load_controllers.launch，rm_config包下的会加载所有的控制器，同时也会读取rm_config包下的rm_controllers中对应的机器人类型的配置文件
### config
rm_controllers中的config配置文件原则上需要写好所有对应的控制器的配置文件，不同的控制器有不同的参数需求
打开控制器的时候can接口应该要能够收发我们特有的控制符200/eff
通用的需要注意的点:
type   对应到launch文件中的controller_loader中的args名称，供launch找到对应的ControllerType
joint  与urdf中写的joint名称相对应
