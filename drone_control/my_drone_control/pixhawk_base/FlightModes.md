# Flight Modes

PX4为用户设定了多种不同的飞行模式，不同飞行模式下对用户输入（这个输入可以来自遥控器也可以来自地面站）的响应不同，进而影响用户对于无人机的控制。

## Position Mode(Multicopter)

**Position**是最容易飞行的一种模式，推荐新手进行飞行。

在**Position**模式下操作杆被释放/居中时，无人机将进行自动控制（通过自主制动、自主调平）使得无人机在3D空间中保持稳定，相当于自主补偿风力以及其它外力对机身平衡带来的影响。同时，在全摇杆偏转的情况下，无人机会开始以 MAV_ACC_HOR_MAX 的加速度进行加速，加速途中逐渐减小加速度，直到最后达到最终速度 MPC_VEL_MANUAL。

由于**Position**模式下无人机会自动保持当前位置而不是继续前进或停止，所以对于新手而言这也是最适合、最安全的飞行模式。

同时**Position**模式需要GPS和遥控器手动操作。

下面这张图直观地显示了**Position**模式下的操作响应：

![Position](https://docs.px4.io/main/assets/img/position_MC.c6191ec9.png)

### Landing

在**Position**模式下着陆也是非常简单的：

1. 使用右摇杆确保无人机水平着陆
2. 放开左、右摇杆使无人机在水平面上空静止
3. 轻轻下拉油门杆使无人机慢慢靠近地面直到接触
4. 将油门杆拉到底方便无人机对着陆进行检测
5. 此时无人机将会自动检测并电机会逐渐停止旋转

注意：

1. 如果无人机在着陆后电机没有停止旋转，可能是因为着陆点不水平，此时锁定车辆后打开```kill switch```模式来停止电机，注意即使在打开```kill swirch```后如果没有对电机上锁，关闭```kill switch```后电机仍然会继续旋转
2. Position模式下即使着陆后电机依然无法上锁，此时应当切换为其它模式对电机进行上锁
3. 如果无人机在着陆后仍然有其它动作，在排除误操作的前提下，着陆后应该重新校准飞控以及相关设备

### Takeoff

在着陆情况下，将油门杆推到62.5%（从油门杆底部到顶部）及以上无人机会起飞。

### Parameters

|    Parameter     |                   Description                    |
| :--------------: | :----------------------------------------------: |
|   MPC_HOLD_DZ    | 启用保持位置的摇杆区域（0.1代表全摇杆范围的10%） |
| MPC_Z_VEL_MAX_UP |             最大上升速度（默认3m/s）             |
| MPC_Z_VEL_MAX_DN |             最大降落速度（默认1m/s）             |
| MPC_ACC_HOR_MAX  |                  最大水平加速度                  |
|  MPC_VEL_MANUAL  |                   最大水平速率                   |
|  MPC_LAND_SPEED  |            着陆下降速率（默认0.7m/s）            |

### Technical Summary

**Position**模式下的飞行十分友好，注意操作不要太大，把控飞行高度和速度，像操作玩具飞机一样操作即可。

---

## Altitude Mode(Multicopter)

**Altitude**模式是一种相对容易的飞行模式，但建议新手先熟悉**Position**模式再尝试在**Altitude**模式下飞行。

在**Altitude**模式下当操作杆被释放/居中时，无人机会保持水平和高度，但是不会自主平衡风力和其它阻力对无人机带来的影响，也就是无人机会向着风向漂移。同时如果无人机处于移动状态时操作杆被释放/居中，无人机会继续行驶直到动量被阻力消散。

**Altitude**模式是最安全的无GPS手动模式，它在释放操作杆时帮助我们锁定了高度，在熟悉了**Position**模式后可以开始尝试此模式下的飞行。

下面这张图直观地显示了**Altitude**模式下的操作响应：

![Altitude](https://docs.px4.io/main/assets/img/altitude_control_mode_copter.8e02e8ec.png)

### Parameters

|    Parameter     |       Description        |
| :--------------: | :----------------------: |
| MPC_Z_VEL_MAX_UP | 最大上升速度（默认3m/s） |
| MPC_Z_VEL_MAX_DN | 最大降落速度（默认1m/s） |

###  Technical Summary

**Altitude**模式下的飞行需要飞手使用roll、pitch操作杆（右边操作杆）来手动平衡阻力。

---

## Manual/Stabilized Mode(Multicopter)

**Manual/Stabilized**模式是一种相对困难的飞行模式，对飞手的操作有一定的要求，建议在**Altitude**模式下熟练飞行后再开始尝试在**Manual/Stabilized**模式下飞行。

在**Manual/Stabilized**模式下当操作杆被释放/居中时，无人机将会停止（高度无法自主保持）且水平（这个水平并不能自主保持），无人机将依靠飞手的操作来平衡阻力以及保持高度。

下面这张图直观地显示了**Manual/Stabilized**模式下的操作响应：

![Manual/Stabilized](https://docs.px4.io/main/assets/img/manual_stabilized_MC.857b681e.png)

### Parameters

|   Parameter   |                        Description                        |
| :-----------: | :-------------------------------------------------------: |
| MPC_THR_HOVER | 当油门杆居中时且MPC_THR_CURVE设置为默认值时输出的悬停油门 |
| MPC_THR_CURVE |                       定义油门缩放                        |

### Technical Summary

**Manual/Stabilized**模式下的飞行需要飞手使用roll、pitch操作杆来手动平衡阻力以及使用油门杆来控制高度。

---

# Flight Modes Setup

PX4允许将飞行模式映射到无线电频道，这就使得将飞行模式映射到遥控器上的开关、操作杆成为可能。

打开QGC，双击或者在终端中运行下面这条命令：

```shell
./QGroundControl.AppImage
```

进入QGC后，点击左上角"Q"图标 > 选择 ```Vehicle Setup``` > 选择 ```Flight Modes``` ，如下图所示：

![Flight Modes Setup](https://docs.qgroundcontrol.com/master/assets/setup/flight_modes/px4_single_channel.jpg)

QGC允许你为飞行模式选定专门的无线电通道，也就是你可以把飞行模式的选择设置在遥控器上的任何一个操作杆上，最多允许设置6个飞行模式，在大疆官方提供的遥控器上，我们最多可以为无人机设置3个飞行模式，下图为大疆官方遥控器：

![大疆官方遥控器](https://rm-static.djicdn.com/tem/%E9%81%A5%E6%8E%A7%E5%99%A81.jpg)

确保遥控器、飞控、计算机的连接后，打开遥控器，拨动遥控器上任意操作杆，在该界面上应该能够看到对应的无线电通道的移动，设置你熟悉的操作杆并指定飞行模式。

通常我们会将遥控器右上角的拨杆设置为飞行模式通道，大疆官方遥控器上对应的6通道。同时将左拨杆的最上方设置为```kill switch```模式（用于着陆后检查无人机以及应对各种异常情况）。

设置完成后，QGC会为你自动保存每一次更改。你还可以通过在每次飞行前拨动遥控器的操作杆来切换操作模式检查QGC是否输出了切换飞行模式的文本来判断设置是否完成、准确。

到此，飞行模式的设置已经全部完成。
