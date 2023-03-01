# rm_referee

referee是一个用于机器人各部件与裁判系统进行交互的包。以前referee是包含在manual内，现在referee和ui的部分都从manual中分离出来，作为一个单独的包工作。

启动referee之后将意味着机器人上电与否，机器人是否超热量、超功率等都会被裁判系统监视，并且机器人的一些行为也会受到裁判系统的控制。

## 用法

```shell
mon launch rm_config referee.launch
#这个launch将会启动referee相关的节点
```

## 代码架构

referee部分主要分为graph、flash_ui、time_change_ui、referee、trigger_change_ui、ui_base、referee_base几个部分。其中，graph、flash_ui、time_change_ui、trigger_change_ui、ui_base属于ui的部分。剩下的负责与裁判系统交互。

---

### *data*

data.h中定义了一些与裁判系统交互需要的变量，比如功率限制、底盘功率限制、机器人id、客户端id以及机器人是否在线等等...同时还负责接收裁判端发送过来的一些数据并且会对这些数据进行CRC校验（确保这些数据在传输过程中没有损坏）。

其中定义了一个**CapacityData**的结构体，声明了一些变量负责存储有关电容的信息。还定义了一个**Base**类，负责接收并处理裁判系统串口发送的数据，并对其进行CRC校验。

---

### *protocol*

protocol.h中定义了rm_referee与裁判系统交互时的协议（就是告诉referee将如何读懂裁判系统发来的数据），其中采用枚举以及结构体的方式定义了这些“协议”（类似于告诉你数字1代表机器人什么状态数字2又代表什么状态），这些“协议”会在referee相关部分运行时使用到。

其中，**GraphOperation**定义了三种图传的操作状态（添加、更新以及删除）；**GraphColor**定义了图形几种颜色，根据需要选择相应颜色；**GraphType**定义了图形的形状，这里决定了你画的ui是一条线还是一个矩形...。

---

### *graph*

graph中封装好了各种API，这些API会被各种ui的类调用，其中***disPlay***函数就是开始画ui，***sendUi***负责将画好的ui发送到裁判端，***getColor、getType、initPosition、updatePosition***等负责初始化好各种参数，这些在**Graph**这个类初始化时调用，也就是这个类初始化时将各种参数设置好了。

