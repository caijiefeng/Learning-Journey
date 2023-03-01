# glog

glog是google开源的一个实现应用程序日志记录的C++库。

## 基本使用

glog最基本的使用如下。

```cpp
#include<glog/logging.h>
//使用glog需要包含这个头文件

google::InitGoogleLogging(argv[0]);
//初始化库，生成日志文件前调用

google::ShutdownGoogleLogging();
//结束glog时需要关闭，否则会造成内存溢出

FLAGS_log_dir = "c:\\Logs";
//设置日志文件保存的目录，这个路径必须已经存在，否则无法生成日志文件，而且要在初始化库之前调用
```

glog输出分别有4个错误级别。

```cpp
enum SeverityLevel
{
　　google::INFO = 0,
　　google::WARNING = 1,
　　google::ERROR = 2,
　　google::FATAL = 3,
};
//4个级别的枚举

LOG(INFO) << "info test";  //输出一个Info日志
LOG(WARNING) << "warning test";  //输出一个Warning日志
LOG(ERROR) << "error test";  //输出一个Error日志
LOG(FATAL) << "fatal test";  //输出一个Fatal日志，这是最严重的日志并且输出之后会中止程序
//不许哟啊加endl，glog会自动换行
```

glog还可以实现条件输出。

```cpp
LOG_IF(INFO, num_cookies > 10) << "Got lots of cookies";  //当条件满足时输出日志

LOG_EVERY_N(INFO, 10) << "Got the " << google::COUNTER << "th cookie";  //google::COUNTER 记录该语句被执行次数，从1开始，在第一次运行输出日志之后，每隔 10 次再输出一次日志信息

LOG_IF_EVERY_N(INFO, (size > 1024), 10) << "Got the " << google::COUNTER << "th big cookie";  //上述两者的结合，不过要注意，是先每隔 10 次去判断条件是否满足，如果滞则输出日志；而不是当满足某条件的情况下，每隔 10 次输出一次日志信息

LOG_FIRST_N(INFO, 20) << "Got the " << google::COUNTER << "th cookie";  //当此语句执行的前 20 次都输出日志，然后不再输出
```

