# 云台调试流程

## 云台基本部署

先对云台进行基本的部署。

找到rm_description中对应兵种云台的urdf。

1. 首先确保云台的yaw和pitch的link的质量、质心、惯量等基本属性是否设置正确。
2. 接着找到joint标签下的limit属性，根据对应电机的官方手册填写最大输出力矩（effort），最大转速（velocity转化为rad/s），lower/upper给+/-1e10。

  示例如下：

```xml
<limit effort="1.2" velocity="31" lower="-1e10" upper="1e10"/>
```

接着找到云台的transmission。

1. 找到actuator下的mechanicalReduction属性，根据对应电机的官方手册填写相应的传动比，如果机械那边有加其它的传动则去向机械索要对应电机的传动比，不过最好自己要会算。传动比必须填写准确（如果发现电机转向反了，则修改正负号）。

示例如下：

```xml
        <actuator name="yaw_joint_motor">
            <mechanicalReduction>-1</mechanicalReduction>
        </actuator>
```

2. 接着找到joint标签下的offset属性，offset是用来平衡误差的。

示例如下：

```xml
        <joint name="yaw_joint">
            <offset>0.502</offset>
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
```

到此，云台基本部署已经基本完成。

## pid整定

云台的参数配置文件在rm_controls中的rm_config/config/rm_controllers中，找到对应兵种的参数文件对应的云台部分。

云台控制器参数示例如下：

```yaml
  gimbal_controller:
    type: rm_gimbal_controllers/Controller
    yaw:
      joint: "yaw_joint"
      pid: { p: 7.5, i: 0, d: 0.3, i_clamp_max: 0.3, i_clamp_min: -0.3, antiwindup: true, publish_state: true }
    pitch:
      joint: "pitch_joint"
      pid: { p: 20.0, i: 0, d: 0.25, i_clamp_max: 0, i_clamp_min: 0, antiwindup: true, publish_state: true }
    imu_name: "gimbal_imu"
    bullet_solver:
      resistance_coff_qd_10: 0.45
      resistance_coff_qd_15: 1.0
      resistance_coff_qd_16: 0.7
      resistance_coff_qd_18: 0.55
      resistance_coff_qd_30: 3.0
      g: 9.81
      delay: 0.1
      dt: 0.001
      timeout: 0.001
      publish_rate: 50
```

以上是云台控制器的参数配置文件，枪管角解算的部分基本不会需要更改，照搬主仓库上步兵的即可，需要通过调试进行更改的部分主要是yaw和pitch电机的pid，使用基本的pid整定法设置即可，pid调个大概就行。

具体可以按照下面的方法进行pid整定：

1. 将需要调试的电机的pid全部设为0，先给出一个d值，这个值随便给，但是也不要几百几百这样子给，给个10或20就差不多，给到电机开始振动，这时候将d减半，还是振动就继续减半，直到电机不再振动，此时的值就是d。
2. 在云台中，不会使用到i，都是pd算法，所以此时保持d值，给一个比较大的p值（大概50左右），这时候电机会开始振动，重复第一步的方法直到电机不再振动，此时的值就是p。
3. 到这里pid的整定基本完成，后续可以根据图像来进行微调。

## 云台校准

云台校准控制器参数示例如下：

```yaml
  gimbal_calibration_controller:
    type: rm_calibration_controllers/JointCalibrationController
    joint: pitch_joint
    actuator: [ pitch_joint_motor ]
    search_velocity: 6.28
    threshold: 1e-2
    pid: { p: 0.2, i: 0, d: 0.0, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true }
```

以上是云台校准控制器的参数配置文件，需要通过调试进行修改的也基本上只有pid部分，其余的照搬主仓库上步兵的即可。

调试流程如下：

1. 打开云台校准控制器，云台此时会自动进行校准，需要注意的是我们使用的pitch电机一般是3508，而yaw使用的一般是6020，6020不需要进行校准，但是3508需要，所以打开云台校准控制器之后，云台的pitch会自动移动到软限位处，观察pitch是否进行了校准。
2. 校准后打开rviz，观察校准后的实物是否和rviz上显示的位置一致，如果一致，则校准成功。
3. 校准完成后的云台应该能够自由移动，（不要打开云台控制器），此时用手移动云台，观察pitch和yaw是否能够正常移动。
4. 云台校准pid使用的仅仅是p比例控制，如果发现校准后云台出现抖动适当减少p的值，如果云台校准的力度很小可以适当增大p值。

## 软限位设置

云台的软限位设置在urdf中，在rm_description中找到对应兵种的gimbal.urdf.xacro。

云台的urdf示例如下：

```xml
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="threshold" value="0.1"/>
    <xacro:property name="pitch_lower_limit" value="-0.406"/>
    <xacro:property name="pitch_upper_limit" value="0.867"/>
    <xacro:property name="yaw_lower_limit" value="-1.1917"/>
    <xacro:property name="yaw_upper_limit" value="1.138"/>

    <link name="yaw">
        <inertial>
            <mass value="0.568477"/>
            <origin xyz="-0.069988 -0.007741 -0.038882"/>
            <inertia ixx="1.216e-3" ixy="-3.509e-4" ixz="-1.522e-3" iyy="4.338e-3"
                     iyz="-1.557e-4" izz="3.37e-3"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://rm_description/meshes/drone/yaw.stl" scale="0.001 0.001 0.001"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://rm_description/meshes/drone/yaw.stl" scale="0.001 0.001 0.001"/>
            </geometry>
        </collision>
    </link>

    <joint name="yaw_joint" type="revolute">
        <origin xyz="0 0 -0.1"
                rpy="0 0 0"/>
        <dynamics damping="0.0" friction="0.1"/>
        <limit effort="1.2" velocity="31" lower="-1e10" upper="1e10"/>
        <safety_controller k_position="100" k_velocity="0.1"
                           soft_lower_limit="${yaw_lower_limit+threshold}"
                           soft_upper_limit="${yaw_upper_limit-threshold}"/>
        <parent link="base_link"/>
        <child link="yaw"/>
        <axis xyz="0 0 1"/>
    </joint>

    <link name="pitch">
        <inertial>
            <mass value="0.609754"/>
            <origin xyz="0.030701 0.00002 -0.03724"/>
            <inertia ixx="1.704e-03" ixy="-7.08992e-07" ixz="-2.916e-04" iyy="5.587e-03"
                     iyz="-1.02674e-06" izz="4.379e-03"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://rm_description/meshes/drone/pitch.stl" scale="0.001 0.001 0.001"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="package://rm_description/meshes/drone/pitch.stl" scale="0.001 0.001 0.001"/>
            </geometry>
        </collision>
    </link>

    <joint name="pitch_joint" type="revolute">
        <origin xyz="0 0 -0.09" rpy="0 0 0"/>
        <dynamics damping="0.0" friction="0.1"/>
        <limit effort="3.0" velocity="86" lower="${pitch_lower_limit}" upper="${pitch_upper_limit}"/>
        <safety_controller k_position="100" k_velocity="0.1"
                           soft_lower_limit="${pitch_lower_limit+threshold}"
                           soft_upper_limit="${pitch_upper_limit-threshold}"/>
        <parent link="yaw"/>
        <child link="pitch"/>
        <axis xyz="0 1 0 "/>
    </joint>

</robot>
```

调试流程如下：

1. 随后在云台的urdf中最上方的几个宏参数，用手移动云台（pitch/yaw）直到卡到机械限位处，此时在plotjugger中打开joint_state的话题看到此时（pitch/yaw）的位置，将这个数值填写在pitch/yaw_upper/lower_limit处，这里是机械限位的填写。！数值为正的为upper！
2. 将机械限位填写在limit effort中的lower和upper上，设定关节的硬限位。
3. 还有一个宏参数threshold，这个参数决定了软限位的位置，软限位=硬限位+/-threshold，所以根据自己的需要填写一个合适的软限位。一般设为0.1左右。
4. 找到joint标签下的safety controller标签，k_position控制限制joint越过软限位的速度上限，k_velocity控制力矩上限。（对于一个速度v，位置x，速度上限v+，位置上限x+，k_velocity设置为kv，k_position设置为kx，则力矩上限为-kv  * (v - v+)，速度上限为 -kx * (x - x+)。）经过测试，k_position设置为100，k_velocity设置为0.1时已经有很好的表现，所以基本都是这个值。
5. 到此软限位的设置基本完成。