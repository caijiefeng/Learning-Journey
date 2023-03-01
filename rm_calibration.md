# rm_calibration_controller

校准控制器，顾名思义用于给电机校准。部分电机在断电之后零点会发生变化，校准控制器在启动后会以一定速度运动直到电机达到机械限位并且将此时的位置复位为零。

### **硬件接口类型**

*EffortJointInterface*：用于向joint（目标电机）发送力矩指令使其到达校准速度。

*ActuatorExtraInterface*：用于获取当前执行器各种信息，包含偏移量、当前位置、是否停止、是否完成了校准......

### **ROS API**

#### Service

**`is_calibrated`** ([control_msgs/QueryCalibrationState](http://docs.ros.org/en/api/control_msgs/html/srv/QueryCalibrationState.html))：向该服务发送请求，该服务会返回目标joint（目标电机）是否已经完成了校准的响应。

#### Parameters

**`search_velocity`** (double)：joint校准的速度。

**`threshold`** (double)：当joint的速度低于这个值并且持续了一段时间后，joint的状态会从MOVING转为CALIBRATED。

---

## 代码解析

#### joint_calibration_controller.h

```cpp
#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <control_msgs/QueryCalibrationState.h>

namespace rm_calibration_controllers
{
class JointCalibrationController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          rm_control::ActuatorExtraInterface>
{
public:
  JointCalibrationController() = default;
  /** @brief Get necessary params from param server. Init joint_calibration_controller.
   *
   * Get params from param server and check whether these params are set.Init JointVelocityController.Check
   * whether threshold is set correctly.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if init successful, false when failed.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Execute corresponding action according to current calibration controller state.
   *
   * Execute corresponding action according to current joint state. If INITIALIZED, target joint will be set
   * a vel_search_ and countdown_ to move, and switch state to MOVING. If MOVING, target joint will move until
   * current velocity lower than threshold last for a while, and switch state to CALIBRATED. If CALIBRATED,
   * target joint velocity will be set to zero and wait for next command.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;
  /** @brief Switch all of the actuators state to INITIALIZED.
   *
   * Switch all of the actuator state to INITIALIZED in order to restart the calibration.
   *
   * @param time The current time.
   */
  void starting(const ros::Time& time) override;

private:
  /** @brief Provide a service to know the state of target actuators.
   *
   * When requesting to this server, it will return respond about whether target actuators has been calibrated.
   *
   * @param req The request of knowing the state of target actuators.
   * @param resp The respond included the state of target actuators.
   * @return True if get respond successfully, false when failed.
   */
  bool isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                    control_msgs::QueryCalibrationState::Response& resp);
  ros::Time last_publish_time_;
  ros::ServiceServer is_calibrated_srv_;
  //  enum { INITIALIZED, BEGINNING, MOVING_TO_LOW, MOVING_TO_HIGH, CALIBRATED }; for GPIO switch
  enum
  {
    INITIALIZED,
    MOVING_POSITIVE,
    MOVING_NEGATIVE,
    CALIBRATED
  };
  int state_{}, countdown_{};
  double velocity_search_{}, target_position_{}, velocity_threshold_{}, position_threshold_{};
  double positive_position_{}, negative_position_{};
  bool is_return_{}, is_center_{}, returned_{};
  rm_control::ActuatorExtraHandle actuator_;
  effort_controllers::JointVelocityController velocity_ctrl_;
  effort_controllers::JointPositionController position_ctrl_;
};

}  // namespace rm_calibration_controllers
```

#### joint_calibration_controller.cpp

```cpp
#include "rm_calibration_controllers/joint_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool JointCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  velocity_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), controller_nh);
  XmlRpc::XmlRpcValue actuator;
  is_return_ = is_center_ = false;
  controller_nh.getParam("center", is_center_);
  if (!controller_nh.getParam("actuator", actuator))
  {
    ROS_ERROR("No actuator given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator[0]);
  if (!controller_nh.getParam("search_velocity", velocity_search_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("threshold", velocity_threshold_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (velocity_threshold_ < 0)
  {
    velocity_threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  if (controller_nh.hasParam("return"))
  {
    ros::NodeHandle nh_return(controller_nh, "return");
    position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), nh_return);
    if (!nh_return.getParam("target_position", target_position_))
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    if (!controller_nh.getParam("threshold", position_threshold_))
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    is_return_ = true;
    returned_ = false;
  }
  // advertise service to check calibration
  is_calibrated_srv_ = controller_nh.advertiseService("is_calibrated", &JointCalibrationController::isCalibrated, this);
  return true;
}

void JointCalibrationController::starting(const ros::Time& time)
{
  actuator_.setCalibrated(false);
  state_ = INITIALIZED;
  if (actuator_.getCalibrated())
    ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f",
             velocity_ctrl_.getJointName().c_str(), actuator_.getOffset());
}

void JointCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  // TODO: Add GPIO switch support
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        if (!is_center_)
        {
          actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
          actuator_.setCalibrated(true);
          ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
          state_ = CALIBRATED;
          if (is_return_)
            position_ctrl_.joint_.setCommand(target_position_);
          else
          {
            velocity_ctrl_.joint_.setCommand(0.);
            returned_ = true;
          }
        }
        else
        {
          positive_position_ = actuator_.getPosition();
          countdown_ = 100;
          velocity_ctrl_.setCommand(-velocity_search_);
          state_ = MOVING_NEGATIVE;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case MOVING_NEGATIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        negative_position_ = actuator_.getPosition();
        actuator_.setOffset(-(positive_position_ + negative_position_) / 2 + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
        if (is_return_)
          position_ctrl_.joint_.setCommand(target_position_);
        else
        {
          velocity_ctrl_.joint_.setCommand(0.);
          returned_ = true;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case CALIBRATED:
    {
      if (is_return_)
      {
        if ((std::abs(position_ctrl_.joint_.getPosition()) - target_position_) < position_threshold_)
          returned_ = true;
        position_ctrl_.update(time, period);
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
  }
}

bool JointCalibrationController::isCalibrated(control_msgs::QueryCalibrationState::Request& req,
                                              control_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED && returned_);
  resp.is_calibrated = (state_ == CALIBRATED && returned_);
  return true;
}

}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::JointCalibrationController, controller_interface::ControllerBase)
```

