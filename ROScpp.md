# ROScpp

## ros::NodeHandle

```cpp
ros::init(argc,argv,"nodeA");
    //这段代码为你初始化并创造了一个节点“nodeA”
ros::NodeHandle nh;
    //这段代码为你创建了一个NodeHandle类的句柄“nh”，相当于启动了“nodeA”
ros::NodeHandle nh1("ns1");
    /*这段代码为你创建了一个NodeHandle类的句柄“nh1”，不同于“nh”，“nh”是属于“nodeA”所在的命名空间下的，而“nh1”是属于“nodeA”所在的命名空间下的“ns1”命名空间下的*/
ros::NodeHandle nh2(nh1,"ns2");
    /*这段代码创建了“nh1”的子句柄“nh2”，它是属于“nodeA“所在的命名空间下的”ns1“下的”ns2“命名空间下的*/
```

## ros::Subscriber

```cpp
ros::NodeHandle nh1;
ros::Subscriber sub1;
    //这段代码创建了一个订阅者“sub1”
sub1 = nh1.subscibe();
    //这段代码令“sub1”订阅指定的话题并存储在”nh1“句柄中
```

## ROS::BREAK

```cpp
ROS::BREAK();
//中断程序并输出本句所在文件以及行数
```

## ROS::ASSERT

```cpp
ROS::ASSERT();
//检验括号内的条件是否符合，如果失败则中断程序并输出本句所在文件、行数以及条件
```

## ROS::INFO

```cpp
ROS::INFO();
//打印你想要打印的内容
```

## ROS::WARN

```cpp
ROS::WARN;
//打印警告信
```

####
