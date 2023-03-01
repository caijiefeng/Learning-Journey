# rm_manual

rm_manual是一个专门用于与裁判系统交互的软件包，主要是用于向裁判系统发送以及接收大量数据和操作手界面（UI）的绘制。

```shell
roslaunch rm_manual load.launch
```

## 配置文件

下面以DynamicX队内无人机的manual参数配置文件示例：

```yaml
rm_manual:
  robot_type: "drone"
  supply_frame: "supply_frame"
  gimbal:
    topic: "/controllers/gimbal_controller/command"
    max_yaw_vel: 12.56
    max_pitch_vel: 8.37
    track_timeout: 0.5
    target_cost_function:
      k_pos: 1.0
      k_vel: 1.0
      k_hp: 1.0
      k_freq: 1.0
      timeout: 0.05
  shooter:
    topic: "/controllers/shooter_controller/command"
    gimbal_error_limit: 0.05
    heat_limit:
      expect_shoot_frequency_1: 20
      expect_shoot_frequency_2: 20
      expect_shoot_frequency_3: 20
      burst_shoot_frequency: 20
      safe_shoot_frequency: 20
      heat_coeff: 8.0
      type: "ID1_17MM"
  detection_switch:
    fail_limit: 10
    service_name: "/detection_nodelet/status_switch"

  controllers_list:
    state_controllers:
      - controllers/joint_state_controller
      - controllers/robot_state_controller
    main_controllers:
      - controllers/orientation_controller
      - controllers/gimbal_controller
      - controllers/shooter_controller
    calibration_controllers:
      - controllers/trigger_calibration_controller
      - controllers/gimbal_calibration_controller
#这个控制器列表中显示了在manual中的三种控制器类型，joint_state以及robot_state属于state_controllers，校准控制器属于calibration_controllers，剩下的属于main_controllers。

  gimbal_calibration:
    - start_controllers:
        - controllers/gimbal_calibration_controller
      stop_controllers:
        - controllers/gimbal_controller
      services_name:
        - /controllers/gimbal_calibration_controller/is_calibrated
  shooter_calibration:
    - start_controllers:
        - controllers/trigger_calibration_controller
      stop_controllers:
        - controllers/shooter_controller
      services_name:
        - /controllers/trigger_calibration_controller/is_calibrated

  ui:
    trigger_change:
      - name: "chassis"
        config: { start_position: [ 400, 750 ], size: 15, width: 2, title: "chassis: " }
      - name: "target"
        config: { start_position: [ 400, 700 ], size: 15, width: 2, title: "target:  " }
      - name: "exposure"
        config: { start_position: [ 400, 650 ], size: 15, width: 2, title: "exposure:" }
    time_change:
      - name: "capacitor"
        config: { start_position: [ 900, 100 ], size: 15, width: 2, delay: 0.5 }
    fixed:
      - name: "2m"
        config: { type: "line", width: 2, color: "yellow",
                  start_position: [ 925, 435 ], end_position: [ 995, 435 ] }
      - name: "3m"
        config: { type: "line", width: 2, color: "yellow",
                  start_position: [ 935, 420 ], end_position: [ 985, 420 ] }
      - name: "5m"
        config: { type: "line", width: 2, color: "yellow",
                  start_position: [ 948, 400 ], end_position: [ 972, 400 ] }
      - name: "mid"
        config: { type: "line", width: 2, color: "yellow",
                  start_position: [ 960, 400 ], end_position: [ 960, 540 ] }
    flash:
      - name: "spin"
        config: { start_position: [ 850, 750 ], size: 20, width: 2,
                  color: "yellow", content: "please spin!!", delay: 0.8 }
      - name: "cover"
        config: { start_position: [ 830, 700 ], size: 25, width: 2,
                  color: "green", content: "cover open!!", delay: 0.8 }
      - name: "armor0"
        config: { type: "circle", width: 3, radius: 50, color: "yellow", delay: 0.75 }
      - name: "armor1"
        config: { type: "circle", width: 3, radius: 50, color: "yellow", delay: 0.75 }
      - name: "armor2"
        config: { type: "circle", width: 3, radius: 50, color: "yellow", delay: 0.75 }
      - name: "armor3"
        config: { type: "circle", width: 3, radius: 50, color: "yellow", delay: 0.75 }

```



## 代码解析

下面以DynamicX队内无人机的maunal代码示例：

> ```cpp
> #include "rm_manual/drone_manual.h"
> 
> namespace rm_manual
> {
>  DroneManual::DroneManual(ros::NodeHandle& nh) : ManualBase(nh)
>  {
>      ros::NodeHandle gimbal_nh(nh, "gimbal");
>      gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
>      ros::NodeHandle shooter_nh(nh, "shooter");
>      shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
>      ros::NodeHandle ui_nh(nh, "ui");
>      trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
>      time_change_ui_ = new TimeChangeUi(ui_nh, data_);
>      flash_ui_ = new FlashUi(ui_nh, data_);
>      fixed_ui_ = new FixedUi(ui_nh, data_);
>      XmlRpc::XmlRpcValue rpc_value;
>      nh.getParam("gimbal_calibration", rpc_value);
>      gimbal_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
>      gimbal_power_on_event_.setRising(boost::bind(&DroneManual::gimbalOutputOn, this));
>      nh.getParam("shooter_calibration", rpc_value);
>      shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
>      shooter_power_on_event_.setRising(boost::bind(&DroneManual::shooterOutputOn, this));
>      controller_manager_.startCalibrationControllers();
>      left_switch_up_event_.setActiveHigh(boost::bind(&DroneManual::leftSwitchUpOn, this, _1));
>      left_switch_mid_event_.setActiveHigh(boost::bind(&DroneManual::leftSwitchMidOn, this, _1));
>      mouse_left_event_.setActiveHigh(boost::bind(&DroneManual::mouseLeftPress, this));
>      mouse_left_event_.setFalling(boost::bind(&DroneManual::mouseLeftRelease, this));
>      mouse_right_event_.setActiveHigh(boost::bind(&DroneManual::mouseRightPress, this));
>      mouse_right_event_.setFalling(boost::bind(&DroneManual::mouseRightRelease, this));
>  }
> 
>  void DroneManual::run()
>  {
>      ManualBase::run();
>      gimbal_calibration_->update(ros::Time::now());
>      shooter_calibration_->update(ros::Time::now());
>  }
>  /*当遥控器打开时，云台和发射自动校准*/
> 
>  void DroneManual::sendCommand(const ros::Time& time)
>  {
>      gimbal_cmd_sender_->sendCommand(time);
>      shooter_cmd_sender_->sendCommand(time);
>  }
>  /*无人机属于rm的结构只有云台和发射，因此只需要包含云台和发射的command_sender*/
> 
>  void DroneManual::updateRc()
>  {
>      ManualBase::updateRc();
>      gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
>  }/*ch_l_x，ch_l_y指的是遥控器左操作杆信号值，x大小定义了操作杆左右移动的幅度，y大小定义了操作杆上下移动的幅度，这里表示上下移动左操作杆使云台俯仰，左右移动左操作杆使云台偏航*/
>  void DroneManual::updatePc()
>  {
>      ManualBase::updatePc();
>      gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x * gimbal_scale_, data_.dbus_data_.m_y * gimbal_scale_);
>  }/*m_x，m_y指的是鼠标的信号值，x大小定义了鼠标左右摆动的幅度，用于使云台偏航，y大小定义了鼠标上下摆动的幅度，用于使云台俯仰*/
> 
>  void DroneManual::gimbalOutputOn()
>  {
>      ManualBase::gimbalOutputOn();
>      gimbal_calibration_->reset();
>  }
> 
>  void DroneManual::shooterOutputOn()
>  {
>      ManualBase::shooterOutputOn();
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
>      shooter_calibration_->reset();
>  }
>  /*OutputOn是接收到上电指令后执行的函数，会重新给云台和发射校准*/   
> 
>  void DroneManual::remoteControlTurnOff()
>  {
>      ManualBase::remoteControlTurnOff();
>      gimbal_calibration_->stop();
>      shooter_calibration_->stop();
>  }
>  /*当遥控器关闭时，main_controllers和校准控制器关闭，意味着此时只有joint_state和robot_state打开，云台和发射的校准停止，进入PASSIVE状态*/
> 
>  void DroneManual::remoteControlTurnOn()
>  {
>      controller_manager_.stopCalibrationControllers();
>      ManualBase::remoteControlTurnOn();
>  }
>  /*当遥控器打开时，main_controllers打开，同时停止校准，机器人进入IDLE状态*/
> 
>  void DroneManual::checkReferee()
>  {
>      ManualBase::checkReferee();
>      gimbal_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
>      shooter_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_);
>  }
>  /*检查裁判系统，根据读取到的裁判系统发来的消息来确定是否给云台和发射上电*/
> 
>  void DroneManual::checkKeyboard()
>  {
>      ManualBase::checkKeyboard();
>      mouse_left_event_.update(data_.dbus_data_.p_l);
>      mouse_right_event_.update(data_.dbus_data_.p_r);
>  }
>  /*检查PC端操作，同时实时更新鼠标左右键的响应*/
> 
>  void DroneManual::drawUi(const ros::Time& time){}
> 
>  void DroneManual::rightSwitchDownRise()
>  {
>      ManualBase::rightSwitchDownRise();
>      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      gimbal_cmd_sender_->setZero();
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
>  }
>  /*当遥控器右拨杆打到下方时，操作模式切换为IDLE，gimbal进入RATE模式，云台指令归0，shooter进入STOP模式*/
> 
>  void DroneManual::rightSwitchMidRise()
>  {
>      ManualBase::rightSwitchMidRise();
>      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
>  }
>  /*当右拨杆打到中间时，操作模式切换为RC，gimbal进入RATE模式，shooter进入STOP*/   
>  void DroneManual::rightSwitchUpRise()
>  {
>      ManualBase::rightSwitchUpRise();
>      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
>  }
> /*当右拨杆打到上方时，操作模式切换为PC，gimbal进入RATE模式，shooter进入STOP*/
> //简单来说右拨杆是用来切换操作模式的。    
>  void DroneManual::leftSwitchDownRise()
>  {
>      ManualBase::leftSwitchDownRise();
>      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
>  }
> /*左拨杆打到下方时，gimbal进入RATE模式，shooter进入STOP*/
>  void DroneManual::leftSwitchMidRise()
>  {
>      ManualBase::leftSwitchMidRise();
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
>  }
> /*左拨杆打到中间时，shooter进入READY，此时摩擦轮会开始转动*/
>  void DroneManual::leftSwitchMidOn(ros::Duration duration)
>  {
>      if (data_.track_data_.id == 0)
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      else
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
>  }
> 
>  void DroneManual::leftSwitchUpRise()
>  {
>      ManualBase::leftSwitchUpRise();
>      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
>  }
> /*左拨杆打到上方时，gimbal进入TRACK模式*/
>  void DroneManual::leftSwitchUpOn(ros::Duration duration)
>  {
>      if (data_.track_data_.id == 0)
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      else
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
>      if (duration > ros::Duration(1.))
>      {
>          shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
>          shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
>      }
>      else if (duration < ros::Duration(0.02))
>      {
>          shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
>          shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
>      }
>      else
>          shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
>  }
> /*当左拨杆打到上方并且保持该位置时，shooter进入PUSH*/
>  void DroneManual::mouseLeftPress()
>  {
>      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
>      if (data_.dbus_data_.p_r)
>          shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
>  }
> /*鼠标左键点击后shooter进入PUSH*/
>  void DroneManual::mouseRightPress()
>  {
>      if (data_.track_data_.id == 0)
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
>      else
>      {
>          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
>          gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
>      }
>  }
> /*鼠标右键点击后进入自瞄*/
> 
> }  // namespace rm_manual
> ```
>

