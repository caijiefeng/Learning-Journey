# [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

### What is VINS-Fusion

VINS-Fusion是一个多传感器融合的解决方案，可用于实现自主应用（无人机、AR/VR、汽车）的准确定位。它基于VINS-Mono并进行了全面的拓展，支持多种视觉惯性传感器（摄像头+IMU），甚至也支持仅视觉传感器。你还可以将VINS与GPS融合来获取更精确的定位。

VINS-Fusion具有以下特点：

* 支持多传感器
* 实时空间校准（相机与IMU之间数据纠正）
* 实时时间校准（相机与IMU之间数据校准）
* 视觉闭环

VINS-Fusion是一个完在[KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)基础上的全开源顶级算法

### Build VINS-Fusion

在rm_workspace下编译VINS-Fusion：

```shell
cd ~rm_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ..
catkin build
```

