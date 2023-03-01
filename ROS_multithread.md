# ROS中的多线程

ROS在使用过程中，经常需要订阅各种消息或者处理各种队列，ROS默认本身是单线程的，如果处理时间不长理论上是可以满足需求的，但是在程序执行过程中难免会遇到一个循环执行的服务需要接收另一个服务信号才会终止其进程的情况，这时候单线程的模式就无法满足我们的需求了。

当整个程序的线程被阻塞，也就是服务的请求需要处理完成后才会处理下一个服务请求，无法实现我们的逻辑。好在ROS为我们提供了多线程的模式。

## ROS自带的多线程指令

### ros::MultiThreadedSpinner

MultiThreadedSpinner类似于ros::spin()，在构造过程中可以指定它所用线程数，但如果不指定线程数或者线程数设置为0，它将在每个cpu内核开辟一个线程。

```cpp
ros::MultiThreadedSpinner spinner(4); // Use 4 threads
spinner.spin(); // spin() will not return until the node has been shutdown
```

### ros::AsyncSpinner

AsyncSpinner比MultiThreadedSpinner更优，它有start() 和stop() 函数，并且在销毁的时候会自动停止。

```cpp
ros::AsyncSpinner spinner(4); // Use 4 threads
spinner.start();
ros::waitForShutdown(); //将4个进程同时循环
```

### 多线程用法例子

```cpp
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

using namespace std;
class service_two
{
    public:
    service_two();
    bool photo_flag;
    ros::ServiceServer service_once;
    ros::ServiceServer service_on;
    private:
    bool get_img(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);
    bool set_flag(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
};
service_two::service_two()
{
    ros::NodeHandle n;
    ros::NodeHandle nh;
    photo_flag = true;  
    service_once = n.advertiseService("photo_once", &service_two::get_img,this);
    service_on = n.advertiseService("photo_on", &service_two::set_flag,this);
}
bool service_two::get_img(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
{
    int cnt = 0;
    while (1)
    {
      if (cnt > 399) {
        break;
      }      
      cnt +=1;
      std::cout<<"while photo_flag:"<<cnt<<std::endl;      
      sleep(1);
      if(photo_flag==false)
          break;
    }
  return true;
}

bool service_two::set_flag(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res)
{
  if (req.data){
    photo_flag=true;
  }else{
    photo_flag=false;
  }
  std::cout<<"photo_flag:"<<photo_flag<<std::endl;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "idmvs_server");
  service_two service_two;
  //启动多线程只要启动下面三行代码就可以了
  ros::AsyncSpinner spinner(2);           //非阻塞式的spinner, 可以使用start和stop进行启停
  spinner.start();                        //启动线程
  ros::waitForShutdown();                 //等待退出
  ros::spin();                            //这个应该可以省略
  return 0;
}

```

