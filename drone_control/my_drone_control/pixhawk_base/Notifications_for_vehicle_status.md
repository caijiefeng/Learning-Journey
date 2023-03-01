# Vehicle Status Notifications

PX4为我们提供了LED、Buzzer方便我们对车辆是否可以起飞以及车辆状态作出判断。

## LED Meanings

Pixhawk系列飞控使用LED向我们提示车辆当前状态。

UI LED 向我们提供了飞行准备相关的车辆状态信息。

Status LED 向我们提供了 PX4IO 和 FMU SoC的状态，这些指示了电源、引导加载程序模式以及行动的信息和错误。

### UI LED

RGB UI LED通常是指安装在GPS上的LED。

下图显示了LED和车辆状态之间的关系：

![LED meanings](https://docs.px4.io/main/assets/img/pixhawk_led_meanings.d8221e50.gif)

下面对这些状态分别进行说明：

* 蓝色常亮：车辆已经布防且无GPS位置锁定，电机解锁，此时你可以驾驶无人机，注意大型螺旋桨在告诉旋转时可能会很危险，所以要十分小心。同时次模式下车辆无法进行制导任务。
* 蓝色闪烁：车辆未布防，没有GPS锁定，此时无法控制电机，但其它所有系统仍在工作。
* 绿色常亮：车辆布防，GPS位置锁定。此时你可以驾驶无人机并让其执行制导任务。
* 绿色闪烁：车辆未布防，GPS位置锁定。此时无法控制电机，但其它所有系统包括GPS位置锁定都在正常工作。
* 紫色常亮：故障安全模式，此时车辆在飞行中，出现问题，车辆将自动尝试返回起飞位置或下降到当前位置。
* 黄色常亮：电池电量警告，当电池电量继续下降，车辆会进入故障安全模式，但当黄色LED灯常亮时，你该结束本次飞行了。
* 红色闪烁：错误，通常情况下表示你的飞控需要校准/配置，连接QGC对飞控完成校准、设置，如果完成后仍然如此，则需要重新检查是否存在另外一个错误。

发生错误时，请在QGC中检查相关信息，包括此刻的校准状态以及 Preflight Checks 等报告的错误信息。还要检查Pixhawk是否正确读取GPS，GPS发送的位置信息是否正确，GPS的连接和安装是否正确等等。

### Status LED

Status LED 有三个为FMU SoC提供状态，有三个为 PX4 IO 提供状态，如下图所示：

![Pixhawk 4](https://docs.px4.io/main/assets/img/pixhawk4_status_leds.d352ada4.jpg)

从给飞控上电开始，FMU 和 PX4 IO 的CPU会先引导加载程序（BL），然后运行程序（APP），下表显示了LED指示状态如何表示Bootloader和APP的使用情况：

| Color     | Label                       | Bootloader usage                               | APP usage               |
| --------- | --------------------------- | ---------------------------------------------- | ----------------------- |
| Blue      | ACT (Activity)              | Flutters when the bootloader is receiving data | Indication of ARM state |
| Red/Amber | B/E (In Bootloader / Error) | Flutters when in the bootloader                | Indication of an ERROR  |
| Green     | PWR (Power)                 | Not used by bootloader                         | Indication of ARM state |

---

## Tune Meanings

Pixhawk系列飞控使用 tones/tunes 和LED来指示车辆状态和事件。

这些 tunes 都被定义在了 /lib/tunes/tune_definition.desc 而且可以使用 tune-control 模块进行调试，你可以通过 ```TUNE_ID_name``` 这个格式来寻找想要的 tunes。

### Boot/Startup

在启动过程中你可能会听到这些音频：

* Startup Tone

<audio id="Startup Tone" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/1_startup_tone.0ccaeec9.mp3"
</audio>

  这代表microSD卡已经成功安装

* Error Tune

<audio id="Error Tune" controls="" preload="none">
<source id="mp3" src="https://docs.px4.io/main/assets/media/2_error_tune.328eda9c.mp3">
</audio>

这可能表示硬件故障导致和系统重新启动；系统设置为使用PX4IO但并不存在IO；UAVCAN已启用但其驱动无法启动；SITL/HITL已启用但*pwm_out_sim*驱动无法启动；FMU启动失败。

* Make File System

<audio id="Make File System" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/16_make_fs.c8bb124e.mp3">
    <audio/>

这可能表示格式化microSD卡；挂载失败；没有microSD卡。

* Format Failed

<audio id="Format Failed" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/17_format_failed.cf47ae6e.mp3">
</audio>

这表示格式化microSD卡失败。

* Program PX4IO

<audio id="Program PX4IO" controls="" preload="">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/18_program_px4io.5a68a888.mp3">
</audio>

这表示开始对PX4IO进行编程。

* Program PX4IO Success

<audio id="Program PX4IO Success" controls="" preload="none">
<source id="mp3" src="https://docs.px4.io/main/assets/media/19_program_px4io_success.898f5c0b.mp3">
</audio>

这表示PX4IO编程已完成。

* Program PX4IO Fail

<audio id="Program PX4IO Fail" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/20_program_px4io_fail.89749277.mp3">
</audio>

这可能表示PX4IO编程失败；PX4IO无法启动；无法找到AMU Mixer。

### Operational

在正常操作期间，你可能会听到这些音频：

* Error Tune

<audio id="Error Tune" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/2_error_tune.328eda9c.mp3">
</audio>

丢失RC信号（遥控器信号）。

* Notify Positive Tone

<audio id="Notify Position Tone" controls="" preload="none">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/3_notify_positive_tone.406154e9.mp3">
</audio>

这可能表示校准成功；成功切换模式；成功接收命令；安全开关关闭（车辆可以开始布防）。

* Notify Neutral Tone

<audio id="Notify Neutral Tone">
<source id="mp3" src="https://docs.px4.io/main/assets/media/4_notify_neutral_tone.86353d53.mp3">
</audio>

这可能表示任务有效且无警告；空速校准；安全开关打开。

* Notify Negative Tone

<audio id="Notify Negative Tone">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/5_notify_negative_tone.cc788397.mp3">
</audio>

这可能表示校准失败；校准已经完成；任务失效；命令被拒绝；布防/撤防被拒绝；切换模式被拒绝。

* Arming Warning

<audio id="Arming Warning">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/6_arming_warning.cf27f57f.mp3">
</audio>

这表示车辆已经完成布防。

* Arming Failure Tune

<audio id="Arming Failure Tune">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/10_arming_failure_tune.755f3d6e.mp3">
</audio>

这表示布防失败。

* Battery Warning Slow

<audio id="Battery Warning Slow">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/7_battery_warning_slow.a804958e.mp3">
</audio>

这表示对你进行低电量警告。

* Battery Warning Fast

<audio id="Battery Warning Fast">
<source id="mp3" src="https://docs.px4.io/main/assets/media/8_battery_warning_fast.da0e8a39.mp3">
</audio>

这表示对你进行严重低电量警告。

* GPS Warning Slow

<audio id="GPS Warning Slow">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/9_gps_warning_slow.f9b36b31.mp3">
</audio>

这表示对你发出GPS信号弱的警告。

* Parachute Release

这表示将触发降落伞释放。

* Single Beep

<audio id="Single Beep">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/14_single_beep.66b57ba2.mp3">
</audio>

这表示将进行罗盘校准，提示用户开始旋转车辆。

* Home Set Tune

<audio id="Home Set Tune">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/15_home_set_tune.bb49c619.mp3">
</audio>

这表示初始位置已经设定。

* Power Off Tune

<audio id="Power Off Tune">
    <source id="mp3" src="https://docs.px4.io/main/assets/media/power_off_tune.217a0e74.mp3">
</audio>

这表示车辆正在关闭。

---

## Preflight Sensor/Estimator Checks

PX4在预飞行前会执行很多传感器和位姿估算的检查，以确保车辆有足够好的状态来进行布防和飞行。

**所有的预检查错误都会报告在QGC中的 ```PREFLIGHT FAIL``` 消息中。**同时日志中的 ```estimator_status.gps_check_fail_flags```消息还向展示了GPS哪些质量检查失败。

下面列出了错误、可能原因、检查方案以及相关参数：

#### PREFLIGHT FAIL: EKF HGT ERROR

IMU和高度测量数据不一致。

解决方案：执行加速度计和陀螺仪校准并重启车辆，如果仍然出错，请检查高度传感器数据是否有问题。

相关参数：```COM_ARM_EKF_HGT```

#### PREFLIGHT FAIL: EKF VEL ERROR

IMU和GPS检测速度不一致。

解决方案：检查GPS速度数据是否不切实际，如果GPS正常，执行加速度计和陀螺仪校准并重新启动车辆。

相关参数：```COM_ARM_EKF_VEL```

#### PREFLIGHT FAIL: EKF HORIZ POS ERROR

IMU和位置测量数据（GPS、外部视觉）不一致。

解决方案：检查传感器数据是否不切实际，如果传感器正常，执行加速度计和陀螺仪校准并重新启动车辆。

相关参数：```COM_ARM_EKF_POS```

#### PREFLIGHT FAIL: EKF YAW ERROR

陀螺仪估计的偏航角度与磁力计或外部视觉估算的偏航角不一致。

解决方案：检查IMU是否正常，检查磁力计是否正常。

相关参数：```COM_ARM_EKF_YAW```

#### PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS

EKF估算的IMU陀螺仪偏差过大。

#### PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION

不同IMU测量的加速度值不一样。

解决方案：检查IMU。

相关参数：```COM_ARM_IMU_ACC```

#### PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION

不同IMU测量的角速率不一致。

解决方案：检查IMU。

相关参数：```COM_ARM_IMU_GYR```

#### PREFLIGHT FAIL: COMPASS SENSORS INCONSISTENT - CHECK CALIBRATION

不同罗盘的测量值差异过大。

解决方案：重新校准罗盘，检查罗盘方向，排除磁干扰。

相关参数：```COM_ARM_MAG_ANG```

####  PREFLIGHT FAIL: EKF INTERNAL CHECKS

GPS水平/垂直速度、磁偏航、垂直位置传感器的创新（惯性导航估算与传感器测量值之间的差异）幅度过大。

解决方案：检查日志文件中的创新级别来确定原因（在```ekf2_innovations```消息中）。常见的问题有：

1. 磁力计校准不良。重新校准。
2. 启动时的初始冲击以及快速移动导致惯性导航解决方案不好。重启车辆并在5s内尽量减少移动。
3. IMU在预热时漂移。重启PX4，重新校准IMU加速度和陀螺仪校准。
4. 磁干扰。等待或重新供电。