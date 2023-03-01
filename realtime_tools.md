# realtime_tools

realtime_tools是ROS官方提供的一个软件包，可用于给编写C++实时控制器的用户通过硬实时循环在ROStopic上发布消息。这个软件包包含了一系列可用的工具并且都可用于硬实时线程且不会破坏正在进行的实时行为。普通的ROS发布者无法保证实时安全性，使用realtime_tools中包装的实时发布者可以很好地解决这个问题，它会创建一个额外的非实时线程在ROStopic上发布消息。

```cpp
   1 #include <realtime_tools/realtime_publisher.h>
   2 
   3 bool MyController::init(pr2_mechanism_model::RobotState *robot,
   4                         ros::NodeHandle &n)
   5 {
   6   ...
   7 
   8   realtime_pub = new 
   9     realtime_tools::RealtimePublisher<mgs_type>(n, "topic", 4);
  10   return true;
  11 }
  12 
  13 
  14 void MyController::update()
  15 {
  16   if (realtime_pub->trylock()){
  17     realtime_pub->msg_.a_field = "hallo";
  18     realtime_pub->msg_.header.stamp = ros::Time::now(); //和普通的发布者一样设定实时发布的时间戳
  19     realtime_pub->unlockAndPublish(); //解锁并且发布消息
  20   }
  21   ...
  22 }
```

