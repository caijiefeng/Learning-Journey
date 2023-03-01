# 编写控制器

## rm_controller

每一个控制器都是一个完整的ROS软件包，它应该包括：

* ```cfg```：存放```.cfg```格式文件
* ```config```：存放```.yaml```格式配置文件，与urdf交互
* ```include```：存放```.h```格式头文件
* ```src```：存放```.cpp```格式文件，也是控制器功能实现的核心代码存放区
* ```launch```：存放```.launch```格式启动文件
* ```README.md```：对整个软件包进行概述的```markdown```文档
* ```package.xml```：描述软件包属性的```xml```文件
* ```CMakeLists.txt```：使用CMake编译

下面的编写教程均采用dynamicx开源的rm_gimbal_controller作示范。

---

## CMakeLists编写

要想控制器可以被编译并执行，CMakeLists的编写不可缺少并且第一步就要执行的任务。

```cmake
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_msgs
        rm_common

        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
        effort_controllers
        tf2
        tf2_eigen
        tf2_geometry_msgs
        visualization_msgs
        dynamic_reconfigure
        angles
        )
# 这里注明了编译这个软件包所需要的库和依赖，告诉CMake这个软件包的编译所需

generate_dynamic_reconfigure_options(
        cfg/BulletSolver.cfg
)
# 将你的软件包内.cfg文件路径添加进来

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        roslint

        rm_msgs
        rm_common

        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
        effort_controllers
        tf2
        tf2_eigen
        tf2_geometry_msgs
        visualization_msgs
        dynamic_reconfigure
        angles
        LIBRARIES ${PROJECT_NAME}
)
# catkin会根据这些库和依赖编译这个软件包，请确保与find_package中所添加的保持一致

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
# catkin会根据你的添加创建include目录

add_library(${PROJECT_NAME} src/gimbal_base.cpp src/bullet_solver.cpp)
# catkin会根据你的添加创建可执行文件

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
# 将可执行文件与目标库链接
```

---

## package.xml编写

要想他人更好地了解软件包，package.xml的编写也是必不可少。

```xml
<name>rm_gimbal_controllers</name>
<version>0.1.6</version>
<!-- 软件包名字、版本 -->

<description>RoboMaster standard robot Gimbal controller</description>
<!-- 尽可能简明地描述你的软件包 -->

<maintainer email="liaoqiayuan@gmail.com">Qiayuan Liao</maintainer>
<author email="liaoqiayuan@gmail.com">Qiayuan Liao</author>
<!-- 该软件包的作者以及维护者 -->

<depend>roscpp</depend>
<depend>roslint</depend>
<!-- 描述该软件包的依赖应该与CMakeLists中保持一致 -->
```

---

## controller.h编写

为了使代码结构更加清晰、简洁，头文件的编写需要遵守一定原则。

```cpp
#include <realtime_tools/realtime_publisher.h>
//include软件包外部的头文件时使用<>而不是""

namespace rm_gimbal_controller
{
struct Config
{
rm_control::RobotStateInterface  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, dt, timeout;
    //将参数保存到配置文件中，而不是将任意的变量存放到配置文件中
};  //创建一个Config结构体保存、定义你需要的参数

class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
hardware_interface::ImuSensorInterface,hardware_interface::EffortJointInterface>
    /*自定义控制器均来自controller_interface中的类，允许至多声明4个接口*/
{
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    //init函数负责参数初始化以及生成接口和句柄
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
    //update函数包含每一个控制周期需要执行的代码
  void setDes(const ros::Time& time, double yaw_des, double pitch_des);
//自定义类控制器中必须声明init、update函数，starting、stopping函数可选
private:
  void rate(const ros::Time& time, const ros::Duration& period);
  void track(const ros::Time& time);
  void direct(const ros::Time& time);
  bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                       const urdf::JointConstSharedPtr& joint_urdf);
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  double feedForward(const ros::Time& time);
  void updateChassisVel();
  void commandCB(const rm_msgs::GimbalCmdConstPtr& msg);
  void trackCB(const rm_msgs::TrackDataConstPtr& msg);

  rm_control::RobotStateHandle robot_state_handle_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  bool has_imu_ = true;
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

  BulletSolver* bullet_solver_{};

  // ROS Interface
  ros::Time last_publish_time_{};    //从ROS官方的作用域下获取时间
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_;
  ros::Subscriber cmd_gimbal_sub_;
  ros::Subscriber data_track_sub_;   //创建订阅者，订阅的话题在cpp中决定
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rm_msgs::TrackData> track_rt_buffer_;

  rm_msgs::GimbalCmd cmd_gimbal_;
  rm_msgs::TrackData data_track_;
  std::string gimbal_des_frame_id_{}, imu_name_{};
  double publish_rate_{};
  bool state_changed_{};

  // Transform
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_;

  // Gravity Compensation
  geometry_msgs::Vector3 mass_origin_;
  double gravity_;
  bool enable_gravity_compensation_;

  // Chassis
  double k_chassis_vel_;
  geometry_msgs::Twist chassis_vel_;

  enum
  {
    RATE,
    TRACK,
    DIRECT
  };   //枚举方式定义3种模式
  int state_ = RATE;  
    //根据需求将控制器的核心功能实现的所有定义都放在private作用域下
};
    
}  //为你的控制器创建一个命名空间，这个命名空间的名称应该与控制器的名称一致
```

---

## controller.cpp编写

控制器的核心功能实现，可执行文件近乎完全决定了控制器所能实现的程度，代码的复用率、精简度以及控制的精度在这里不会提及，只作为控制器可执行文件的编写教程。类似于头文件的编写，可执行文件的编写除了考虑功能的可实现性之外同样需要遵守相应的原则。

```cpp
#include "rm_gimbal_controllers/gimbal_base.h"
//include自定义的头文件应该使用""而不是<>

namespace rm_gimbal_controllers  //自定义控制器的实现应写在自定义控制器命名空间下
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  publish_rate_ = getParam(controller_nh, "publish_rate", 50);

  config_ = { .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
              .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
              .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
              .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
              .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
              .g = getParam(controller_nh, "g", 0.),
              .delay = getParam(controller_nh, "delay", 0.),
              .dt = getParam(controller_nh, "dt", 0.),
              .timeout = getParam(controller_nh, "timeout", 0.) };
    //使用ROS官方封装的getParam函数从配置文件中获取参数
  config_rt_buffer_.initRT(config_);
    //用realtimetools官方封装的initRt函数更新/初始化参数
}
    
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  bool enable_feedforward;
  enable_feedforward = controller_nh.getParam("feedforward", xml_rpc_value);
  if (enable_feedforward)
  {
    ROS_ASSERT(xml_rpc_value.hasMember("mass_origin"));
    ROS_ASSERT(xml_rpc_value.hasMember("gravity"));
    ROS_ASSERT(xml_rpc_value.hasMember("enable_gravity_compensation"));
      //使用ROS_ASSERT()检验条件，条件不符程序会停止并且输出所在文件、行数以及条件
  }
  mass_origin_.x = enable_feedforward ? (double)xml_rpc_value["mass_origin"][0] : 0.;
  mass_origin_.z = enable_feedforward ? (double)xml_rpc_value["mass_origin"][2] : 0.;
  gravity_ = enable_feedforward ? (double)xml_rpc_value["gravity"] : 0.;
  enable_gravity_compensation_ = enable_feedforward && (bool)xml_rpc_value["enable_gravity_compensation"];

  k_chassis_vel_ = getParam(controller_nh, "yaw/k_chassis_vel", 0.);
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
    //使用ros::NodeHandle创建子句柄
  bullet_solver_ = new BulletSolver(nh_bullet_solver);
    //在init函数中对自定义控制器分出的类进行init操作

  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
    //初始化接口
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
    return false;
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
      //使用ROS_INFO输出你想输出的内容
  }    
    
  gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name + "_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.; 
    //与urdf内定义的坐标系的交互
    
  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  data_track_sub_ = controller_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Controller::trackCB, this);
     //使用nh.subscribe令节点订阅相应的topic
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100));
    
return true;  //以上执行完毕后，init完毕返回true    
}    /*init函数根据需要编写，这会初始化参数并创建所需要的节点、句柄*/
    
void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;
  state_changed_ = true;
}  /*staring函数根据需要编写，在dynamicx的gimbal_controller中开启云台控制器将会默认进入RATE模式*/
    
void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
    //update函数需要利用realtime对订阅者所订阅到的消息进行更新
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", ctrl_pitch_.joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", ctrl_yaw_.joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());  //使用ROS_WARN来发布警告信息
    return;
  }
  updateChassisVel();
  if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case RATE:
      rate(time, period);
      break;
    case TRACK:
      track(time);
      break;
    case DIRECT:
      direct(time);
      break;
  }//利用switch case对每个模式进行处理，这是控制器的核心
  moveJoint(time, period);
} /*update函数每一个控制周期执行一次*/    
}

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
    //将自定义控制器类注册并发布为一个插件
```

---

## controller.launch编写

```xml
<arg name="robot_type" default="$(env ROBOT_TYPE)" doc="Robot type [standard, hero, engineer]"/>
<!-- 读取环境变量并启动本质就是读取对应机器人的urdf和配置文件 -->

<rosparam file="$(find rm_gimbal_controllers)/config/$(arg robot_type).yaml" command="load"/>
<!-- 读取配置文件并加载参数进服务器 -->

<node name="controller_loader" pkg="controller_manager" type="controller_manager" output="screen" args="load controllers/gimbal_controller"/>
<!-- 将可执行文件/cpp文件初始化为节点并启动节点 -->
```

