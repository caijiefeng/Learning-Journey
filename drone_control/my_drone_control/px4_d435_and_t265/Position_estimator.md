# 位姿估算

## VIO、MoCap系统

VIO（视觉惯性里程计）和MoCap（运动捕捉）系统可以被应用在GPS不可用或者不可靠时导航车辆（室内、桥下等），VIO、MoCap两个系统都是依靠“视觉”上的信息来确定车辆的姿态，不同的是，VIO使用机载传感器（通常是相机）获取车辆的角度进而确定位姿信息，而MoCap是利用机载传感器来捕获车辆在3D空间中的位姿，可以说VIO是以第一视角来确定车辆位姿，而Mocap是以上帝视角来确定。

这两个系统所获取的位姿数据都可以用于PX4更新车辆实时的位姿信息，也可以将这些获取到的数据融合到位姿算法中，也就是既可以用于提供也可以用于改进。

---

## 位姿信息在PX4中集成

PX4会将从VIO、Mocap中获取到的位姿数据集成到合适的MAVLink上，再通过飞控上的MAVLink处理模块转换为```uORB```消息。

下面展示了用于给PX4获取位姿信息的MAVLink消息，同时还展示了PX4将这些MAVLink消息映射到对应的```uOBR```主题：

|                           MAVLink                            |             uORB              |
| :----------------------------------------------------------: | :---------------------------: |
| [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)[ ](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) | ```vehicle_visual_odometry``` |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)(`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)） | ```vehicle_visual_odometry``` |
| [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) | ```vehicle_mocap_odometry```  |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)(`frame_id =` [MAV_FRAME_MOCAP_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_MOCAP_NED)) | ```vehicle_mocap_odometry```  |

以上这些消息都应该以30Hz和50Hz之间的频率传输，如果频率太低，EKF2不会融合外部视觉信息。

**PX4使用EKF2算法/LEP算法来获取/融合位姿信息**

需要注意的是EKF2算法仅订阅```vehicle_visual_odometry```主题，只能处理 VISION_POSITION_ESTIMATE 以及 ODOMETRY(```frame_id```=MAV_FRAME_LOCAL_FRD)这两条MAVLInk消息 ，而MoCap系统必须生成这些消息才能被EKF2使用。LPE算法可以订阅这两个主题，可以处理上述的所有消息。

但是EKF2算法是PX4默认使用的算法，有更加好的测试、支持，所以尽管LPE估算能订阅更多的主题，处理更多的消息，还是更加优先使用EKF2算法。事实上我们也只会考虑使用EKF2。

当前版本的PX4并不支持以下几种MAVLink视觉消息：

* [GLOBAL_VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#GLOBAL_VISION_POSITION_ESTIMATE)
* [VISION_SPEED_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE)
* [VICON_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VICON_POSITION_ESTIMATE)

---

## 参考坐标系

* FRD坐标系（X-Forward、Y-Right、Z-Down）
* NED坐标系（X-North、Y-East、Z-Down）
* FLU坐标系（X-Forward、Y-Left、Z-Up）
* ENU坐标系（X-East、Y-North、Z-Up）

PX4从不同传感器获得的坐标系所根据的是不同的参考坐标系，需要进行坐标系的转换以使其符合PX4的约定，这种坐标系的转换可以通过在QGC中设置参数也可以通过修改软件包（VIO···）中的配置文件来实现，也可以通过编写程序来转换。

### 通过代码转换

下面就显示通过修改代码来实现坐标系转换：

```cpp
x_{mav} = x_{mocap};
y_{mav} = z_{mocap};
z_{mav} = - y_{mocap};
```

### 通过EKF2配置转换

打开QGC，点击左上角Q图标，按顺序选择**Vehicle Setup > Parameters > EKF2**，下面展示了需要配置的参数：

|                          Parameter                           |           Setting for External Position Estimation           |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| [EKF2_AID_MASK](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_AID_MASK) |              根据需要选择EKF2融合位姿信息的来源              |
| [EKF2_HGT_MODE](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_HGT_MODE) |              根据需要选择EKF2获取高度信息的来源              |
| [EKF2_EV_DELAY](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EV_DELAY) | 设置捕获时间（机载传感器）以及测量时间（飞控内置）之间的差异 |
| [EKF2_EV_POS_X](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EV_POS_X), [EKF2_EV_POS_Y](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EV_POS_Y), [EKF2_EV_POS_Z](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_EV_POS_Z) |               设置机载传感器到车辆的坐标系转换               |

配置完成后需要重新启动飞控，这些参数才会生效。

### 通过启动文件转换

***建议先阅读后面的部分再看此部分，可以更好地理解***

这里以VIO示例，VIO通过mavros来桥接，你怎么安装VIO的传感器决定了EKF2获取的坐标系来源，这种坐标系的转换在mavros中通过ROS的tf来实现，所以在启动文件中配置好tf是十分必要的。

在VIO包中的`bridge_mavros.launch`文件中：

```xml
  <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="0 0 0 <yaw> <pitch> <roll> base_link <external_pose_child_frame> 1000"/>

<!-- 使用tf把camera_link映射到base_link上，这个映射关系由参数列表中的六个数值决定，分别代表xyz以及yaw、pitch、roll -->

<node pkg="tf" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
        args="0 0 0 <yaw> <pitch> <roll> odom <external_pose_parent_frame> 1000"/>

<!-- 使用tf把camera_odom_link映射到odom上，这个映射关系由参数列表中的六个数值，分别代表xyz以及yaw、pitch、roll，如果camera_odom_link的z轴指向上方，则yaw、pitch、roll设为0即可 -->
```

注意在使用mavros的odom插件来发布tf时，不允许由其它的ROS节点在发布这两个坐标之间的tf，因为这可能会破坏tf树。

---

## 与ROS交互

ROS并不能够为PX4提供外部的位姿信息，但是ROS已经非常完善地完成与VIO和MoCap的集成。

ROS虽然不能直接为PX4提供外部位姿信息，但是ROS可以收集外部的位姿信息并反馈给PX4，其中MAVROS在这之中扮演了非常重要的角色。ROS在飞控与相机之间的沟通充当着一个“翻译”/“传递”，下面显示了MAVROS“翻译”/“传达”信息的通道：

|                          ROS Topic                           |                           MAVLink                            |             uORB              |
| :----------------------------------------------------------: | :----------------------------------------------------------: | :---------------------------: |
|                   /mavros/vision_pose/pose                   | [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE)[ ](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) | ```vehicle_visual_odometry``` |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)（`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)） | ```vehicle_visual_odometry``` |
|                      /mavros/mocap/pose                      | [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) | ```vehicle_mocap_odometry```  |
| /mavros/odometry/out (`frame_id = odom`, `child_frame_id = base_link`) | [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)（`frame_id =` [MAV_FRAME_LOCAL_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_FRD)） | ```vehicle_mocap_odometry```  |

LPE算法支持上面4种通道，EKF2仅支持视觉通道，想要EKF2使用来自MoCap捕获到的数据则需要对位姿话题进行[重映射](http://wiki.ros.org/roslaunch/XML/remap)。

---

## 参考坐标系转换

PX4、ROS所使用的参考坐标系并不相同，哪怕是对于PX4和ROS单独而言，它们对于机身坐标系和世界坐标系的认定也是不相同的：

* PX4使用的机身坐标系为FRD坐标系，世界坐标系为FRD或者NED坐标系。
* ROS使用的机身坐标系为FLU坐标系（也就是ROS中最常见的前左上），世界坐标系使用的是FLU或者ENU坐标系。

我们使用的是相机给PX4提供位姿信息，所以在相机的ROS驱动中，我们会将相机在ROS中的```camera_link```映射到机身在ROS中的```base_link```，再通过转换将`base_link`映射到PX4中的机身坐标（也就是航天中用到的坐标）。同样ROS中的世界坐标系也是通过这种映射到PX4中。

下面这张图显示了PX4和ROS（左/右）选用的参考坐标系：

![PX4/ROS参考坐标系](https://docs.px4.io/main/assets/img/ref_frames.b0d97b5d.png)