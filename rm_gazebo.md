# rm_gazebo

gazebo是ros官方提供的一个集成了运动学算法、机器人模型、碰撞属性的仿真环境，大多数基于ros的程序都是在gazebo上进行仿真运行模拟真实环境的，对于大多数ros的使用者来说，gazebo是首选的第一个仿真环境。

rm_gazebo在gazebo的基础上定义了robomaster的赛场环境，使得我们可以在场地受限或者没有实车的时候在gazebo中模拟出我们需要的赛场环境，检测我们程序的有效性。

### gazebo的组成

#### world描述文件

world顾名思义就是世界，也就是这个文件定义了我们需要的世界环境，它包含了仿真器中需要的所有元素，包含机器人、灯光、传感器以及静态对象等，world文件使用了SDF格式，一个典型的特点文件的拓展名为`.world`。

gazebo服务器（gzserver）通过读取world文件来生成一个世界。

#### model文件

model文件也一样使用的是SDF格式，但只包含一个标签<model>...</model>，这个文件的目的是为了更好地重复使用模型和简化world文件，使用model文件只需要在world文件中include想要使用的model文件即可。

