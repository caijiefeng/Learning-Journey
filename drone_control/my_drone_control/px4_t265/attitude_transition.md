# PX4中的姿态转换

## PX4中常用的几种坐标系

地理坐标系（g系）：

一般来说无人机的姿态都是定义在地理坐标系下的，常见的地理坐标系有“东北天”和“北东地”两种，航天用的一般是北东天。

![地理坐标系](https://img-blog.csdnimg.cn/4439eab65d724200a9ee73ed9174f9f8.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAeHVqdW45MjU=,size_11,color_FFFFFF,t_70,g_se,x_16#pic_center)

机身坐标系（b系）：

机身坐标系的坐标原点设置在机身的质心处，可以简化运动学方程的解算、推导，x轴指向机身前进方向，y轴指向机身右侧，z轴指向地面，符合右手定则，也就是我们常说的前右下坐标系。

![b系](https://img-blog.csdnimg.cn/d7712969389a43c9863e72eff54cd986.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAeHVqdW45MjU=,size_20,color_FFFFFF,t_70,g_se,x_16)

导航坐标系（n系）：

导航坐标系在水平面上的投影即为地理坐标系。

## 无人机的姿态转换

描述无人机的姿态转换可以用欧拉角、方向余弦矩阵、四元数表示。

欧拉角（偏航角 ψ、俯仰角 θ 和滚转角 φ）：

在b系中，偏航角是值无人机绕着z轴旋转产生的角度，俯仰角是值绕着y轴旋转产生的角度，滚转角是绕着x轴旋转产生的角度。

![无人机欧拉角](https://img-blog.csdnimg.cn/2b6a87bed1604c2094ca8e5cea92b023.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAeHVqdW45MjU=,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center)

方向余弦矩阵（旋转矩阵）：

方向余弦矩阵可以用来计算本体坐标系相对于全局坐标系中的相对方位，对于它的基本定义和解算过程可以看下面这个博客的文章，很详细https://www.cnblogs.com/andychenforever/p/6298073.html。仔细看完后也就可以理解为什么机体的欧拉旋转可以用方向余弦矩阵表示了。

![三个方向余弦矩阵表示三次欧拉旋转](https://img-blog.csdnimg.cn/17c8f2c21cb14395b2b0368daf4726ab.png#pic_center)

![](https://img-blog.csdnimg.cn/17c8f2c21cb14395b2b0368daf4726ab.png#pic_center)

![](https://img-blog.csdnimg.cn/b8aa1ce7359842f7b60d2caeeda77645.png#pic_center)

将这三个方向余弦矩阵相乘就可以得到机身坐标系到地理坐标系的旋转。

![](https://img-blog.csdnimg.cn/8bb3f43b7b9549768b3d1663d52a4dd9.png#pic_center)

所以当我们知道机身坐标系任一向量的坐标，通过方向余弦矩阵就可以得到其在地理坐标系中的坐标，反过来也可以。

四元数：

用四元数来表示旋转有着得天独厚的优势，但是四元数相比欧拉角和方向余弦矩阵更加难以理解，下面有篇四元数的文章真的太细了写得非常值得看https://www.qiujiawei.com/understanding-quaternions/。

![](https://img-blog.csdnimg.cn/626e0cabc191455894b08771497bb2e5.png#pic_center)

## 欧拉角、方向余弦矩阵及四元数之间的转换关系

方向余弦矩阵用欧拉角/四元数表示：

![](https://img-blog.csdnimg.cn/60363136b35c4e769c91c7c3e8689d2f.png#pic_center)

欧拉角表示四元数：

![](https://img-blog.csdnimg.cn/a96d009d202c4fa4b7cc45b90e1eefe9.png#pic_center)

四元数表示欧拉角：

![](https://img-blog.csdnimg.cn/9c0b9b932d7d4e539c3afeac3d5b879b.png#pic_center)

方向余弦矩阵表示四元数：

![](https://img-blog.csdnimg.cn/e14503b03b984b8d9c0eaf02e2b25f48.png#pic_center)