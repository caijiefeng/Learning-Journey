# 关于Linux的进程优先级

对于一个成熟的多任务操作系统LInux来说，进程人为可控是一个很有必要的事情，因为不同的进程有不同的重要性，我们对于人为控制进程占用CPU有着强烈的需求。

所以进程优先级就出现了。进程优先级起作用的方式不论是从单个CPU时代，还是多核CPU时代都是通过控制进程占用CPU时间长短来实现的。也就是说，优先级高的进程占用CPU时间更长，优先级低的进程占用CPU的时间更短。

## nice

我们最熟悉的Linux设置进程优先级的方式估计肯定就是nice、renice命令了。nice值是一个反映进程优先级的值，取值范围为[-20，19]，一共40个级别，值越小进程优先级越高，值越大，进程优先级越低。

```shell
nice -n 10 bash
#通过nice命令对将要执行的命令赋nice值，上面这段命令打开了一个bash并且将其nice值设置为10（默认的nice值为0）

nice
#查看当前shell的nice值

#同样，使用ps、top查看进程的nice值也是可行的
ps -l
    F S   UID   PID  PPID  C PRI  NI ADDR SZ WCHAN  TTY          TIME CMD
    4 S     0  6924  5776  0  80   0 - 17952 poll_s pts/5    00:00:00 sudo
    4 S     0  6925  6924  0  80   0 -  4435 wait   pts/5    00:00:00 bash
    0 R     0 12971  6925  0  80   0 -  8514 -      pts/5    00:00:00 ps
    
top
  PID USER      PR  NI  VIRT  RES  SHR S %CPU %MEM    TIME+  COMMAND                                                             
 3001 root      20   0  232m  21m 4500 S 12.9  0.0   0:15.09 python                                                             
11541 root      20   0 17456 2400  888 R  7.4  0.0   0:00.06 top   ##可以看到每一个进程都具有自己的PRI值和nice值（ps下看到的PRI、NI，top下看到的PR、NI），尽管PRI值和nice值都反映了进程优先级，但是nice值和PRI值不是一个东西，所以我们就把nice值理解为一个能反映进程优先级但不是进程值（PRI）的东西就行了  
```

记住“越nice的人往往越不会抢占资源，越不nice的人越会抢占资源”，所以nice值高反而优先级低。

nice值也叫静态优先级，因为一个进程一旦设定了nice值就不会再改变（除非人为使用renice命令修改），但是PRI值却会在内核的O1调度器上改变，所以PRI值也叫动态优先级。

## 优先级和实时进程

在内核中，进程优先级的取值范围是一个宏（MAX_PRIO）定义的，它的值为140，这个值又是通过另外两个值相加而来的，一个是代表nice值取值范围的宏（NICE_WIDTH），另一个是代表实时进程优先级范围的宏（MAX_RT_PRIO）。所以说，Linux定义了140个优先级范围[0，139]，还是值越小，优先级越高，而nice值的[-20，19]映射过来就是优先级范围的[100，139]。

```shell
define DEFAULT_PRIO            (MAX_RT_PRIO + NICE_WIDTH / 2)
#新进程的默认优先级将会被这样定义，实际上对应的就是nice值为0的时候，就算我们使用nice/renice命令修改其nice值，这个进程的PRI值也不会超过[100，139]，除非是实时进程
```

以上我们已经大概了解了非实时进程的优先级，那么接下来了解实时进程的优先级。在Linux中，进程分为实时进程和非实时进程两种，其中非实时进程的算法主要依赖于O1和CFS的进程调度算法，而实时进程由于要保证在短时间内响应以及要求最小的中断延时和进程切换延时，O1和CFS两种进程调度算法都无法满足，所以Linux在设计时候，专门为实时进程映射了100个优先级[0，99]，也就是说，实时进程的优先级要高于非实时进程的优先级，采用的调度算法也更简单以此来减少不必要的开销。

实时进程和非实时进程的主要区别就是通过优先级来判断的，所有优先级在[0，99]内的都是实时进程，在[100，139]内的都是非实时进程。

```shell
#使用chrt命令来查看、设置一个实时进程的优先级状态
chrt
Show or change the real-time scheduling attributes of a process.
Set policy:
 chrt [options] <priority> <command> [<arg>...]
 chrt [options] -p <priority> <pid>
Get policy:
 chrt [options] -p <pid>
Policy options:
 -b, --batch          set policy to SCHED_OTHER
 -f, --fifo           set policy to SCHED_FIFO
 -i, --idle           set policy to SCHED_IDLE
 -o, --other          set policy to SCHED_OTHER
 -r, --rr             set policy to SCHED_RR (default)
Scheduling flag:
 -R, --reset-on-fork  set SCHED_RESET_ON_FORK for FIFO or RR
Other options:
 -a, --all-tasks      operate on all the tasks (threads) for a given pid
 -m, --max            show min and max valid priorities
 -p, --pid            operate on existing given pid
 -v, --verbose        display status information
 -h, --help     display this help and exit
 -V, --version  output version information and exit
For more details see chrt(1).
#关注policy options部分，系统给出了5种调度策略，用于实时进程的有SCHED_FIFO、SCHED_RR（这两者的区别在于FIFO采用先进先出的策略，优先级一样的情况下先执行的先调度直到退出才会调度下一个进程，而RR是以时间片轮转的方式对优先级相同的多个线程同时进行处理），用于非实时进程的有SCHED_OTHER、SCHED_OTHER、SCHED_IDLE。
```

