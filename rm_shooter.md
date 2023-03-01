# **RmShooter**
## **README**
rm\_shooter\_controller有四种状态:STOP、READY、PUSH、BLOCK,shooter通过PID算法来对发射器左右摩擦轮和触发轮进行控制,同时设置摩擦轮的角速度来设置子弹速度,还可以实现挡块的检测

### **使用到的硬件接口类型**
*JointStateInterface*
用于获取摩擦轮和触发轮（扳机）的速度以及触发轮的位置
*EffortJointInterface*
用于发送摩擦轮和触发轮的扭矩命令

### **用法**
***mon launch rm_shooter_controllers load_controllers.launch***

### **Cfg**
*shooter.cfg*
在这之中添加不同子弹速度所对应的摩擦轮角速度等相关参数以及扳机block状态检测的相关参数

### **Launch files**
基本上所有的controller使用的都是load\_controlllers.launch

### **ROS上的API接口**
**订阅话题**
command （rm\_msgs/ShootCmd）:有关于控制器状态、子弹速度以及射击频率的一系列命令

**相关参数**
*block_effort、block_speed、block_duration*
当版机的扭矩大于block\_effort所设定的数值,且角速度小于block\_effort设定值,而且持续的时间超过了block\_duration设定的数值那么发射器的状态会被切换为**BLOCK**
*blcok_overtime*
如果发射器的控制器进入**BLOCK**状态的时间超过了block\_overtime设定的时间,发射器控制器的状态会切换为**PUSH**
*anti_block_angle*
对于进入**BLOCK**状态的shooter\_controller,摩擦轮将会尝试反转anti\_block\_angle所设定的角度来尝试脱离**BLOCK**状态,如果成功脱离,shooter\_controller将自动切换到**PUSH**状态
*anti_block_threshold*
如果摩擦轮反转的角度超过anti\_block\_threshold所设定的角度,则代表摩擦轮反转成功,将会切换为**PUSH**状态
*qd_10、qd_15、qd_18、qd_30*
表示摩擦轮的角速度,qd后面的数字象征着不同的子弹速度

**Example**
```
shooter_controller:
    type: rm_shooter_controllers/Controller
    publish_rate: 50
    friction_left:
      joint: "left_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    friction_right:
      joint: "right_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    trigger:
      joint: "trigger_joint"
      pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
    push_per_rotation: 8
    push_qd_threshold: 0.90
    block_effort: 0.95
    block_duration: 0.05
    block_overtime: 0.5
    anti_block_angle: 0.2
    anti_block_threshold: 0.1
    qd_15: 460.0
    qd_18: 515.0
    qd_30: 740.0
```

---

## standard.cpp
### **update函数**
对控制器的各种状态进行更新（初始化）
### **stop函数**
当*state_change=true*时,将*state_change调为false*并且shooter的状态切换为**STOP**同时更新左右摩擦轮以及扳机的命令
### **ready函数**
当*state_change=true*时,将*state_change调为false*并且shooter的状态切换为**READY**同时正常初始化
### **push函数**
当*state_change=true*时,将*state_change调为false*并且shooter的状态切换为**PUSH**
*//检测摩擦轮*
如果左右摩擦轮的角速度满足发射条件,向扳机发送shoot的命令并且将当前上一次发射的时间设置为当前时间（last\_shoot\_time\_ = time）
如果左右摩擦轮不满足发射条件,将shooter的状态设置为等待摩擦轮
*//检测扳机*
如果扳机的扭矩小于block\_effort所设定的数值并且扳机的角速度小于block\_speed设定的数值将再次进入检测,如果当前的时间减去进入block\_time的时间大于等于block\_duration所设定的时间（(time - block\_time\_).toSec() >= config_.block\_duration）,那么shooter\_controller将会进入**BLOCK**状态
### **block函数**
当*state_change=true*时,将*state_change调为false*并且shooter的状态切换为**BLOCK**并将上一次进入**BLOCK**状态的时间改为当前时间（last\_block\_time\_ = time）并向扳机发送命令
如果扳机进入**BLOCK**状态的位置减去扳机当前的位置小于anti\_block\_threshold所设定的数值*或者*当前的时间减去上一次进入**BLOCK**状态的时间大于block\_overtime所设定的数值,shooter\_controller的状态也会被切换为**BLOCK**

