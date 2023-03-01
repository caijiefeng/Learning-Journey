# **URDF**
urdf是一种统一的机器人描述文件，它使用xml语言格式

[URDF](http://wiki.ros.org/cn/urdf/Tutorials)
[xacro编写](https://gitlab.com/gdut-yxj/tutorials/ros/xacro)

关于urdf的很多基础知识在上面两个链接中已经描述的非常详细，包括如何编写自己的机器人的urdf，urdf可以说是机器人编程的入门、基础。

## joint safety limits

urdf给每一个joint都提供了safety controller（可写可不写），如果你的joint需要软限位并且需要控制力矩、位置等，那么这个标签可以更好地帮助你。

首先，我们要清楚关节安全限制（保护）的意义和作用，joint的安全限制措施分为力矩限制以及速度限制。力矩限制是通过限制输出的力矩以达到预期的效果，当我们设定了20N-m时，控制器会自动为我们将joint输出的力矩控制在-20N-m到20N-m之间，一旦超过这个限制，那么输出的力矩会被阻断。速度限制本质上是通过限制力矩来达到限制速度的效果，也就是说控制器会根据我们的指令来限制输出的力矩不让joint产生多余的力矩来推动joint超出限制的速度，一旦超出限制的速度，那么joint会应用阻力来进一步达到限制的效果。

下面是一个joint的urdf示例：

```xml
    <joint name="yaw_joint" type="revolute">
        <origin xyz="0 0 -0.1"
                rpy="0 0 0"/>
        <dynamics damping="0.0" friction="0.1"/>
        <limit effort="1.2" velocity="31" lower="-1e10" upper="1e10"/>
<!-- 其中的limit effort设定了限制的力矩，velocity限制了速度的范围rad/s，lower和upper指定了关节的硬上限
        -->
        <safety_controller k_position="100" k_velocity="0.1"
                           soft_lower_limit="${yaw_lower_limit+threshold}"
                           soft_upper_limit="${yaw_upper_limit-threshold}"/>
<!-- k_velocity决定了力矩的限制范围，如果对于一个速度v，速度限制设为v+，k_velocity限制为kv，则力矩最大上限为-kv * (v - v+) -->
<!-- k_position决定了速度的限制范围，如果对于一个位置x，位置限制设为x+，k_position限制为kx，则速度最大上限为 -kx * (x - x+) -->      
        <parent link="base_link"/>
        <child link="yaw"/>
        <axis xyz="0 0 1"/>
    </joint>
```

