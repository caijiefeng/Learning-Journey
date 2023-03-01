# uORB

## 基本概念

uORB又叫微对象处理器，是Pixhawk系统中非常重要的一个模块，它负责着整个系统的数据传输任务，所有的传感器数据都经过它处理后发送给各个模块进行进一步的计算。实际上uORB是一多个进程同时打开一个设备文件，每一个进程上进行数据交互的消息称为topic（主题）。这些进程可以去订阅、发布topic，一个topic可以有多个发布者、订阅者，一个进程也可以订阅多个主题，但是一个topic上只有一种消息。

uorb是被用于线程/进程之间通信的一种以```publish()```/```suscribe()```方式异步传递消息的API接口。在PX4中，uorb是无人机各个模块间通信的协议机制。uorb是一种非常有意思的通信协议，在这个协议的约定下，消息的发布者并不关心谁来接收这个消息也不关心这个消息会被多少人接收，它只关心自己是否能够发布消息；而消息的接收者不关心消息是谁发送的，它也不关心是否把所有消息都接收了，它只关心在自己接收消息的时候是否有消息给它接收。也就是说，uorb协议本质上是一个多对多的消息传输关系，发布者以自己的频率发布消息，订阅者可以随时来获取消息，意思就是如果发布者发布的消息会覆盖掉之前的消息，那么订阅者只会接收到最新的数据。这样子的一个通信协议可以保证飞控各个模块之间互不干扰，相互独立。

```uorb start```：

uORB会在飞控启动的前期自动启动，当然也可以通过```uorb start```来启动它。

```uorb_tests```：

使用```uorb_tests```来进行单元测试。

```uorb top```：

使用```uorb top```来显示每个topic发布的频率，如下所示：

```sh
update: 1s, num topics: 77
TOPIC NAME                        INST #SUB #MSG #LOST #QSIZE
actuator_armed                       0    6    4     0 1
actuator_controls_0                  0    7  242  1044 1
battery_status                       0    6  500  2694 1
commander_state                      0    1   98    89 1
control_state                        0    4  242   433 1
ekf2_innovations                     0    1  242   223 1
ekf2_timestamps                      0    1  242    23 1
estimator_status                     0    3  242   488 1
mc_att_ctrl_status                   0    0  242     0 1
sensor_accel                         0    1  242     0 1
sensor_accel                         1    1  249    43 1
sensor_baro                          0    1   42     0 1
sensor_combined                      0    6  242   636 1
```

这些列分别是：主题名称、多实例索引、订阅者数量、以赫兹为单位的发布频率、每秒丢失消息的数量（对于所有订阅者的组合）和队列大小。

---

## uORB使用

uORB的常用函数（uORB函数返回的是OK/ERROR，类似C++中的0/1）如下：

```cpp
int orb_subscribe(const struct orb_metadata *meta);
//这个函数的用处是订阅主题，可以订阅没有数据的主题但是不能订阅没有发布（不存在）的主题，meta是一种uORB的元对象，可以认为是主题名称
int orb_set_interval(int handle,unsigned interval);
//这个函数用来设置订阅的最小时间间隔，如果设置了那么在这个时间间隔内的数据将订阅不到，其中handle是函数返回的句柄，interval是时间间隔
int orb_copy(const struct orb_metadata *meta,int handle,void *buffer);
//这个函数可以将订阅的主题中获取的数据存进buffer中
orb_advert_t
```

