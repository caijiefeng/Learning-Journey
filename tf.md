# tf

tf(TransForm)是ROS中的坐标转化，tf本质上是一个树状的数据结构，所以也常常称为“tf tree”，也可以用另一种更熟悉的方式来定义——把tf看作一个topic:/tf，而话题中的message就是tf tree的数据结构格式。tf维护了整个机器人以及地图的坐标转换关系。

## tf格式规范

```
std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
string child_frame_id
geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion rotation
        float64 x
        float64 y
        flaot64 z
        float64 w
```

tf中的*header*定义了序号（seq）、时间（stamp）、坐标系名称以及子坐标系的名称。子坐标系和父坐标系的转换就是由```geometry_msgs/Transform```来定义。*translation*中的```Vector3```是三维向量，表示平移。*rotation*中的```Quaternion```是四元数，表示旋转。

最终，许多的```TransformStamped.msg```发向tf，形成了tf树。

## tf相关命令

```shell
rostopic info /tf
# 查看自己的tf版本

rosrun tf view_frames
# 订阅tf并为你绘制一个tf tree的pdf图

rosrun rqt_tf_tree rqt_tf_tree
# 使用rqt动态查询当前tf树

rosrun tf tf_echo[reference_frame][target_frame]
# 查看两个坐标系的转换关系
```

## tf静态发布

ROS发布tf有两种方法。

1.在源代码中发布tf转换：

```cpp
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle n;
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;//1.定义一个广播者broadcaster
    tf::Transform transform;//2.声明一个变量用来存储转换信息
    transform.setOrigin(tf::Vector3(0.1, 0, 0.2));//3. 设置坐标原点，（0.1，0，0.2）为子坐标系激光坐标系base_laser在父坐标系小车base_link坐标系中的坐标，
    tf::Quaternion q;// 4.定义旋转
    q.setRPY(0, 0, 0);//（0，0，0）为base_laser在base_link坐标系下的roll(绕X轴)，pitch(绕Y轴)，yaw(绕Z轴) 的旋转度数，现在都是0度
    transform.setRotation(q);
    while(n.ok()) {
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "", "base_laser"));  //将变换广播出去，发布了base_link和base_laser之间的坐标关系
        r.sleep();
    }
 
    return 0;
}

//静态发布tf2，现在tf2已经全面取代了tf
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");
  if(argc != 8)
  {
    ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if(strcmp(argv[1],"world")==0)
  {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;

  }
  static_turtle_name = argv[1];
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = static_turtle_name;
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
  ros::spin();
  return 0;
};
```

2.在launch文件中定义广播者：

```xml
<node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom 100" />

<!--其中name为自己定义的，args分别代表了 x y z yaw pitch roll，声明了odom到map的坐标转换关系，100代表100ms发布一次-->
```

在终端中启动静态tf发布节点：

```shell
rosrun tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

## tf动态发布

对于固定的坐标系我们可以采用静态发布的方式，但是如果一个坐标系一直在运动那么当我们在发布该坐标系的tf时就不能单纯地采用静态发布，下面示例动态发布tf：

```cpp
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <iostream>
using namespace std;

std::string turtle_name;
void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);//发布tf，这里的发布与静态发布一致
}
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("turtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
    cout<<turtle_name<<endl;
  }
  else
  {
    private_node.getParam("turtle", turtle_name);
  }
  ros::NodeHandle node;
  //订阅乌龟的位姿，系统发布出来的是小乌龟话题为：/turtle1/pose，键盘：/turtle1/cmd_vel
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
```

## tf监听

通过监听tf可以实现很多我们所知道却不知道如何实现的功能，例如追踪、跟随、移动到指定位置，在ROS中这些都离不开tf的监听。

代码示例：

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;
  ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);//使用服务器来开启另一只小乌龟。

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);//小乌龟的控制指令

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                               ros::Time(0));//监听turtle2到turtle1到变换，不能监听ros::Time::now()，因为tf的建立是需要时间的
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;
    //根据两只小乌龟的相对位姿来定义控制命令
    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
};
```

## 建立新坐标系

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

   tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "turtle1";//建立自己的frame与已知frame之间的tf
  transformStamped.child_frame_id = "carrot1";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
        q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);//发布tf
    rate.sleep();
    printf("sending\n");
  }
};
```

下面这位博客的文章很值得一看，讲的很详细：

https://blog.csdn.net/QLeelq/article/details/111126464

## tf原理

### ROS坐标系规范

ROS的坐标系统一使用右手定义：

![ROS坐标系](https://img-blog.csdnimg.cn/20200408223717567.png)

所以默认情况下，x轴代表前方，y轴代表左方，z轴代表上方。

ROS的绕轴旋转也使用右手定义：

![ROS绕轴旋转](https://img-blog.csdnimg.cn/20200408223958941.png)

所以绕轴旋转时逆时针为正，顺时针为负，也就是机器人向左转为正。

### 欧拉角和四元数的转换

欧拉角就是我们常说的rpy，就是绕x、y、z轴旋转的角度。而四元数是使用一个旋转的向量 + 一个旋转的角度来表示物体旋转。如依次绕z,y,x分别旋转一个固定角度，使用yaw，pitch，roll分别表示物体绕,x,y,z的旋转角度，记为![\psi](https://private.codecogs.com/gif.latex?%5Cpsi)，![\theta](https://private.codecogs.com/gif.latex?%5Ctheta)，![\phi](https://private.codecogs.com/gif.latex?%5Cphi)，可以利用三个四元数依次表示这三次旋转：

![Q_1=cos(\psi /2 ) +sin(\psi /2) k](https://private.codecogs.com/gif.latex?Q_1%3Dcos%28%5Cpsi%20/2%20%29%20&plus;sin%28%5Cpsi%20/2%29%20k)

![Q_2=cos(\theta /2 ) +sin(\theta /2) j](https://private.codecogs.com/gif.latex?Q_2%3Dcos%28%5Ctheta%20/2%20%29%20&plus;sin%28%5Ctheta%20/2%29%20j)

![Q_3=cos(\phi /2 ) +sin(\phi /2) i](https://private.codecogs.com/gif.latex?Q_3%3Dcos%28%5Cphi%20/2%20%29%20&plus;sin%28%5Cphi%20/2%29%20i)

这样我们就可以得到欧拉角与四元数的转换公式：

![img](https://img-blog.csdnimg.cn/2019121716023573.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3hpYW9tYV9iaw==,size_16,color_FFFFFF,t_70)

在代码中实现：

```cpp
struct Quaternion
{
    double w, x, y, z;  //定义四元数
};
 
Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
 
    return q;
}
```

