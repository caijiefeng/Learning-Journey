#**ControllerNamespace命名空间**
每个controller都有自己的namespace,namespace单独拉一个文档来写

***C++的命名空间***
C++命名空间主要用于区分不同库中相同名称的函数、变量、类等等，本质上使用命名空间就是给所使用的函数、变量、类指定了一个范围

定义命名空间使用*namespace*关键字
```C++
namespace namespace_name {
   // 代码声明
}
```
调用命名空间下带有的函数或者变量
```C++
name::code;  // code 可以是变量或函数
```
using指令
```C++
using namespace std
```
命名空间可以定义在不同的文件中，这几个部分共同组成一个完整的命名空间
命名空间还支持嵌套命名
```C++
namespace namespace_name1 {
   // 代码声明
   namespace namespace_name2 {
      // 代码声明
   }
}

// 访问 namespace_name2 中的成员
using namespace namespace_name1::namespace_name2;
 
// 访问 namespace:name1 中的成员
using namespace namespace_name1;
```

***Controller中的命名空间***
*Example*
```C++
namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, dt, timeout;
};

class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);

  bool solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed);
  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw_real, double pitch_real,
                        double bullet_speed);
  double getResistanceCoefficient(double bullet_speed) const;
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  void bulletModelPub(const geometry_msgs::TransformStamped& map2pitch, const ros::Time& time);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  bool dynamic_reconfig_initialized_{};
  double publish_rate_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{};

  geometry_msgs::Point target_pos_{};
  visualization_msgs::Marker marker_desire_;
  visualization_msgs::Marker marker_real_;
};
}  // namespace rm_gimbal_controllers
```
*explicit关键字*声明为explicit的构造函数不能在隐式转换中使用
