# [vision_to_mavros](https://github.com/thien94/vision_to_mavros)

vision_to_mavros是一个免费开源的用于给px和apm使用者实现视觉定位的ros软件包，与px4官方推荐使用的VIO相似，都是使用mavros作为桥接节点将相机的数据流到飞控上，尽管vision_to_mavros是用于给apm使用的，但是大量实验证明在px上也有出色的表现。

相比于VIO，vision_to_mavros的坐标系转换更难以理解（但实际上也不是很难），除此之外两者没有明显区别，定位效果也差不多。使用vison_to_mavros还是VIO都可以，看个人喜好。

### How it works?

***`vision_to_mavros_node`***

这个节点是整个软件包最重要的节点，因为它决定了飞控是否能接收到符合px规范的位姿数据，这个节点会监听相机的数据，并且发布相机位姿到飞控规范的NED坐标系的`tf`转换，这个tf转换主要由下面几个参数控制：

- `target_frame_id`: id of target frame (world/map/base_link)
- `source_frame_id`: id of source frame (camera/imu/body_link)
- `output_rate`: the output rate at which the pose data will be published.
- `roll_cam`, `pitch_cam`, `yaw_cam`, `gamma_world`: angles (in radians) that will convert pose received from `source_frame_id` to body frame, according to ENU conventions.

关于这个坐标转换在vision_to_mavros中的launch文件中已经有了保姆级别的介绍，照着修改即可。

### How we use it?

1. ```shell
   roslaunch realsense2_camera rs_t265.launch
   #启动t265节点
   rostopic echo /camera/odom/sample
   #确保相机在正确运行，移动相机确保其数据正常
   ```

2. ```shell
   roslaunch mavros px4.launch
   #启动mavros节点，看看mavros是否能连接上飞控，修改fru_url
   rostopic echo /mavros/state
   #应该要能够看到connected status为true
   ```

3. ```shell
   roslaunch vision_to_mavros t265_tf_to_mavros.launch
   #启动vison_to_mavros节点，开始传输相机数据
   rostopic echo /mavros/vision_pose/pose
   #确保视觉数据正在被发布
   ```

4. ```shell
   roslaunch vision_to_mavros t265_all_nodes.launch
   #启动所需要的所有节点，这时候你应该能在QGC中看到相应的mavlink视觉消息
   ```