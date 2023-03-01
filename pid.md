# PID
PID是一种经典的闭环控制算法，具有原理简单，容易实现，适用面积广，控制参数相互独立，参数选定简单等优点。一般来说PID算法经常被应用到“保持稳定”的场景中，比如维持平衡、温度稳定以及转速稳定等。
其中：
* P代表比例，它的作用相当于动力，可以类比弹簧弹力，它总是使得物体回复到平衡位置。
* D代表微分，它的作用相当于阻力，它总是在阻止物体远离平衡位置，可以减小系统的超调量。
* I代表积分，有了P的动力和阻力之后，大部分情况下系统都能很快回复到平衡位置，但是假定此时有一个外力与P的动力抵消，而且此时又快要靠近平衡位置了，D的阻力也很小，那此时系统就可能就会在这个非平衡位置的位置平衡下来，为了避免这种情况，I就可以在这时候将从前每一次的误差累积起来，只要存在误差，I就能提供回复平衡位置的力，直到最后平衡为止，可以说I的作用就是在消除系统静态误差。
## 位置式PID
位置式PID是当前系统的实际位置和想要达到的预期位置有偏差，进行PID控制。
![位置式PID](https://pic3.zhimg.com/v2-ba7a2e24948bfb28f39b686283004682_r.jpg)
其代码实现如下：
```cpp
//位置式PID算法代码实现
previous_error = 0; //上一次的误差
integra = 0; //积分项
loop: //根据目标值与测量值循环计算更新输出值（一般是PWM信号）
    error = setpoint - measured_value; //误差项=目标值-输出值
    integra += error * dt; //积分项=从前每一次误差项的累积
    derivative = (error - previous_error) / dt; //微分项=误差的变化率
    output = Kp*error + Ki*integral + Kd*derivative; //输出=三项分别乘比例系数
    previous_error = err; //更新误差
    wait(dt); //等待固定的计算周期
    goto loop; //一般代码里不要使用goto语法，但这里使用便于理解
```
## 增量式PID
增量式PID首先计算的是Δu(k)，然后与上次的输出相加，才是此次的输出结果。增量式PID没有误差累加，控制增量Δu(k)的确定仅与最近3次的采样值有关。
![增量式PID](https://pic4.zhimg.com/v2-031a076e7ce9bc9d17ac86bcf4d6922b_r.jpg)
```cpp
//增量式PID算法代码实现
previous02_error = 0; //上上次偏差
previous01_error = 0; //上次偏差
integral = 0; //积分和
pid_out = 0; //pid增量累加和
loop:
    error = setpoint − measured_value;  /*误差项=目标值-测量值*/
    proportion = error - previous01_error;                            /*比例项=误差项-上次偏差*/
    integral = error * dt;                                            /*积分项=误差项的累计*/
    derivative = (error − 2*previous01_error + previous02_error) / dt;/*微分项=上次误差与上上次误差的变化率*/
    /*或写成：derivative = ( (error − previous01_error)/dt - (previous01_error - previous02_error)/dt )*/
    pid_delta = Kp × error + Ki × integral + Kd × derivative; //计算得到PID增量
    pid_out = pid_out + pid_delta; //计算最终的PID输出
​
    previous02_error = previous01_error; //更新上上次偏差
    previous01_error = error; //更新上次偏差
    wait(dt); //等待固定的计算周期
    goto loop;
```
## PID参数整定
进行PID参数调节时，一般使用试凑法，PID参数整定口诀如下：
> 参数整定找最佳，从小到大顺序查，
> 先是比例后积分，最后再把微分加，
> 曲线振荡很频繁，比例度盘要放大，
> 曲线漂浮绕大湾，比例度盘往小扳，
> 曲线偏离回复慢，积分时间往下降，
> 曲线波动周期长，积分时间再加长，
> 曲线振荡频率快，先把微分降下来，
> 动差大来波动慢，微分时间应加长，
> 理想曲线两个波，前高后低四比一，
> 一看二调多分析，调节质量不会低。
