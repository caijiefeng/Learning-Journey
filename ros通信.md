# ROS中的通信机制
在了解ROS通信机制之前我们先需要了解同步通信和异步通信。

同步通信是指接收端和发送端需要实时沟通，实时响应的通信方式，可以比喻成打电话，接收者必须接通了通讯才会进行。而异步通信指接收端和发送端不需要实时响应，就好像发微信，接收的人可以随时去接收。

这两种通信方式各有优缺点，同步通信时效性强可以快速得到结果，但是它消耗的资源多，性能较差而且需要的耦合度高。异步通信的性能更好，响应更快而且不会造成资源浪费，但是它的架构更复杂，不好管理。

ROS提供了三种通信方式，(topic,service,action)，同步异步都有。

## ROS Topic

ROS中的节点之间是通过话题(topics)来相互通信的，topic是ROS中最常见的一种通信方式，是一种异步收发的通信方式。

ROS还提供了一个节点管理器(ROS Master)，所有的节点都在这个节点管理器上注册，在这个节点管理器上可以查询到信息该发往哪一个节点。每一个节点都会在Master上注册为发布者或者订阅者，发布者发布topic，订阅者去订阅topic上的数据，发布者只管发布一点不关心订阅者什么时候去接收，订阅者也一样不管发布者什么时候发布，它可以随时去获取数据。同时每一个topic上的数据也是有类型的，这个数据类型可以由用户自定义也可以使用ROS提供的标准数据类型。

需要注意的是，一个topic可以有多个发布者和订阅者，而且在topic的通信方式中数据传输是单向的是不可逆的。

topic依托于一个.msg文件，可以理解为一种文本文件，通过编译后会生成相应的可执行文件。

## ROS Service
service是一种基于客户端和服务端的一种同步通信方式，这种通信方式下，允许接收端(客户端Client)主动请求数据，服务端在Master上注册一个service，此时客户端会向服务端发送一个请求，服务端收到请求后处理完成后会给出一个反馈，或者可以说是应答。

service依托于一个与编程语言无关的.srv文件，可以把这个文件理解为一个文本文件，编译后会生成相应的可执行文件。		

##　ROS Action

除了topics和services，ROS还提供了另外一种同步通信方式actions，这种通信方式的出现主要是为了弥补services的不足，当我们有同步通信的要求时而且这个任务的执行时期还非常长，我们需要实时查看进度，action就派上用场了。

action和service的机制很像，但是action具有反馈机制，还允许我们在执行时候终止任务。具体就是客户端给服务端抛出一个目标，服务端接收到目标之后就会开始处理，这个时候客户端可以去做任何事情，但它会周期性地接收来自服务端反馈的进度，如果客户端不满意可以随时终止当前任务。

action也是依托于一个后缀为.action的文件，编译后生成相应的可执行文件。

关于action的代码理解可以看这篇博客的文章：[https://blog.csdn.net/weicao1990/article/details/80322110]		

## 创建ROS消息和服务
### msg和srv
*msg*
msg文件是一个描述ROS消息的文本文件，用于编程语言编写的消息生成源代码
***exanple***
```
uint8 RATE = 0
uint8 TRACK = 1
uint8 DIRECT = 2

time stamp
uint8 mode

# RATE
float64 rate_yaw
float64 rate_pitch

# TRACK/DIRECT
float64 bullet_speed
geometry_msgs/PointStamped target_pos
geometry_msgs/Vector3Stamped target_vel
//消息类型+消息名称
```

*srv*
srv文件描述一个服务，包含了请求和响应
***example***

```
uint8 color
uint8 target
uint8 armor_target
uint8 exposure
uint8 RED = 0
uint8 BLUE = 1
uint8 ARMOR = 0
uint8 BUFF = 1
uint8 ARMOR_ALL = 0
uint8 ARMOR_OUTPOST_BASE = 1
uint8 ARMOR_WITHOUT_OUTPOST_BASE = 2
uint8 EXPOSURE_LEVEL_0 = 0
uint8 EXPOSURE_LEVEL_1 = 1
uint8 EXPOSURE_LEVEL_2 = 2
uint8 EXPOSURE_LEVEL_3 = 3
uint8 EXPOSURE_LEVEL_4 = 4
---
bool switch_is_success
//与msg文件基本一样，不过多了一个---来区分请求(上)和响应(下)
```

msg文件在编译后会在include目录下生成对应的hpp头文件，但其中存储的信息一般要用模板进行封装，srv也是一样
同时cmakelist中也要补全相关内容
[msg和srv](http://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv)
