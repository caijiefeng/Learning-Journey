# MAVLink
MAVLink是一种轻量级的消息传递协议，用于与无人机以及无人机的机载组件之间进行通信。MAVLink遵循混合发布-订阅和点对点的设计模式:数据在topic上发布，而一些任务协议、参数协议采用点对点重传。

MAVLink消息在xml文件中定义，每一个xml文件都定义了一个MAVLink的消息集，类似于ROS在msg文件中定义消息类型，你可以把MAVLink当做人类，而每一个xml文件可以是学生、老师、上班族等等，也可以直接把这些xml文件定义的消息当做各种各样的方言。标准的MAVLink消息集在common.xml中定义，而其它的消息集大部分都是在common.xml的基础上定义的，你也可以自己定义一个新的MAVLink消息。并且这些xml定义的消息都经过了代码生成器处理，生成了特定编程语言（例如C++、C、python、java等）的软件库，这些软件库大多数都经过了MIT许可，可以不受限制地使用，无人机、地面控制站和其它MAVLink系统通过这些软件库进行通信。

MAVLink的主要优点:
* 高效率
-MAVLink 1仅8字节，包含丢包检测和开始符号。MAVLink 2仅14字节，更安全且可拓展。而且MAVLink不需要额外成帧。
* 可靠
-MAVLink提供了检测数据包丢失、损坏和数据包身份的验证。
* 支持多种不同编程语言，可在多种操作系统上使用
* 网络上允许最多255个系统
* 允许机外与机上通信

MAVLink的数据帧格式如下：

![MAVLink](https://img-blog.csdnimg.cn/fb8d47b9661c40588f02f16fc26ac9b6.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAeHVqdW45MjU=,size_20,color_FFFFFF,t_70,g_se,x_16)
