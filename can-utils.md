#**can-utils**
can-utils是一个针对can通信的测试工具,通常来说canutils中包含5个独立的程序:canconfig、candump、canecho、cansend、cansequence

canconfig:用于配置CAN总线接口的参数（波特率和模式）
Ccandump:从CAN总线接口接收数据并以16进制形式打印到标准输出
canecho:把从CAN总线接口接收到的所有数据重新发送到CAN总线接口
cansend:向指定的CAN总线接口发送指定的数据
cansequence:向指定的CAN总线接口重复*递增数字*或者指定接收模式并且检验接收的*递增数字*

---

**常用的命令:**
1.***sudo ip link set can0 up type can bitrate 1000000***
配置can0
2.***candump can0***
接收can总线上发送的数据,用于检验can总线是否正常通信
3.***cansend can0 -e 标识符***
向can总线上发送信息,标识符有两种，分别是1FF和200，两种标识符对应两组电机,16位数字每4位对应一个电机的电流大小.共四个电机,即使电机不足四个也要补满16位;由此可见一根can线最多挂载八个电机

