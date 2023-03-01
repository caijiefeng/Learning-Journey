# C++

这里补充一些看代码时候遇到的C++知识。

---

## std::thread

*thread*称为线程，也可以说是分类编程技术。

多线程技术可以让计算机在同个时间内执行多个线程，大大提高了效率和整体的性能。需要注意的是线程与进程是两个不同的概念，一个程序有且只有一个进程，但是一个程序可以同时拥有多个线程。

下面这张图片显示了进程与线程之间的关系：

![进程与线程](https://img-blog.csdnimg.cn/20210717195132759.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NqY18wOTEw,size_16,color_FFFFFF,t_70)

### std::thread常用成员函数

**构造&析构函数**

|                             函数                             |      类型      |                   作用                   |
| :----------------------------------------------------------: | :------------: | :--------------------------------------: |
|                      thread() noexcept                       |  默认构造函数  |         创建一个线程，什么也不做         |
| template <class Fn, class… Args>explicit thread(Fn&& fn, Args&&… args) | 初始化构造函数 | 创建一个线程，以`args`为参数执行`fn`函数 |
|                thread(const thread&) = delete                |  复制构造函数  |            创建并删除一个线程            |
|                 thread(thread&& x) noexcept                  |  移动构造函数  |  构造一个与`x`相同的对象,会破坏`x`对象   |
|                          ~thread()                           |    析构函数    |                 析构对象                 |

**成员函数**

|              函数              |                             作用                             |
| :----------------------------: | :----------------------------------------------------------: |
|          void join()           |               等待线程结束并清理资源（会阻塞）               |
|        bool joinable()         |           返回一个bool值用于确定是否执行joint函数            |
|         void detach()          | 将线程与调用其的线程分离，彼此独立执行（此函数必须在线程创建时立即调用，且调用此函数会使其不能被join） |
|    std::thread::id get_id()    |                          获取线程id                          |
| thread& operator=(thread &&rhs | 见移动构造函数（如果对象是joinable的，那么会调用`std::terminate()`结果程序） |

### 使用实例

**Example1**

```cpp
#include <iostream>
#include <thread>
using namespace std;
void doit(){cout << "World!";}
int main()
{
    thread a({cout << "Helllo,";});
    thread b(doit);
    a.join();
    b.join();
    return 0;
}
/*可能产生的结果
Hello,World! && World!Hello,
这是因为多线程进行时是以异步方式进行的，相当于每个线程都在赛跑，谁跑赢谁就先执行完毕 */
```

**Example2**

```cpp
#include <iostream>
#include <thread>
using namespace std;
template<class T> void changevalue(T &x, T val) {
	x = val;
}
int main() {
	thread th[100];
	int nums[100];
	for (int i = 0; i < 100; i++)
		th[i] = thread(changevalue<int>, ref(nums[i]), i+1);
	for (int i = 0; i < 100; i++) {
		th[i].join();
		cout << nums[i] << endl;
	}
	return 0;
}
/* 这个程序执行的结果是从1-100依次输出，std::ref 可以包装按引用传递的值，std::cref 可以包装按const引用传递的值，这两个包装的操作解决了线程传递时需要右值传递的操作。 */
```

### 注意事项

* 线程是在thread对象被定义的时候开始执行的，而不是在调用join函数时才执行的，调用join函数只是阻塞等待线程结束并回收资源。
* 分离的线程（执行过detach的线程）会在调用它的线程结束或自己结束时释放资源。
* 线程会在函数运行完毕后自动释放，不推荐利用其他方法强制结束线程，可能会因资源未释放而导致内存泄漏。
* **没有执行`join`或`detach`的线程在程序结束时会引发异常**。

---

## std::mutex

*mutex*是互斥量，使用时需要include<mutex>头文件。

在多线程同时进行的时候，如果出现了多个线程同时操作一个变量的情况，系统就会出错，所以C++11提供了std::mutex来让线程锁定，直到这个线程将mutex解锁后才允许其它线程调用。

### std::mutex常用成员函数

此处的``mutex``代指需要操作的对象。

|      函数       |                             作用                             |
| :-------------: | :----------------------------------------------------------: |
|   void lock()   | 将mutex上锁。如果mutex已经被其它线程上锁，那么会阻塞，直到解锁；如果mutex已经被同一个线程锁住，那么会产生死锁。 |
|  void unlock()  | 解锁mutex，释放其所有权。如果有线程因为调用lock()不能上锁而被阻塞，则调用此函数会将mutex的主动权随机交给其中一个线程；如果mutex不是被此线程上锁，那么会引发未定义的异常。 |
| bool try_lock() | 尝试将mutex上锁。如果mutex未被上锁，则将其上锁并返回true；如果mutex已被锁则返回false。 |

### 使用实例

```cpp
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;
int n = 0;
mutex mtx;
void count10000() {
	for (int i = 1; i <= 10000; i++) {
		mtx.lock();
		n++;
		mtx.unlock();
	}
}
int main() {
	thread th[100];
	for (thread &x : th)
		x = thread(count10000);
	for (thread &x : th)
		x.join();
	cout << n << endl;
	return 0;
}
```

---

## callback函数

C++中callback函数（回调函数）指的是一个通过函数指针调用的函数，也就是说回调函数一般是在另一个函数调用时被调用的，这个特点也决定了回调函数一般是在多进程的程序中使用，以减少代码量和提高代码效率。

举个简单的例子，在一个程序中同时存在进程A和进程B，这两个进程同时进行，进程A中有A1/2/3这三个函数，进程B中有B1/2两个函数，而A2作为回调函数只有在进程B调用B1函数的时候调用，如下图所示：

![callback函数](https://img-blog.csdnimg.cn/img_convert/03244a22239f62e17889ebab41d9b628.png)

我们当然可以在进程A执行的同时不停地检查进程B是否调用了B1函数来确定A2函数是否需要被调用，但是当代码量很大，程序运行将会消耗非常多的资源、时间，不仅费时费力，更重要的是程序运行的实时性无法被保证，因为我们无法确保一直处于检查状态的进程A是否在第一时间反馈了B1的执行情况，所以回调函数的作用就在这里体现了出来。

同样的情况，我们只需要将A2函数的地址传给B1就可以很好地解决这个问题。

如何获取函数的地址？

```cpp
//普通函数作为回调函数，这种是最简单，但是通常大型的项目、工程中并不会出现可以作为回调函数的普通函数，通常是封装好的类下定义的回调函数
#include <iostream>

void programA_FunA1() { printf("I'am ProgramA_FunA1 and be called..\n"); }

void programA_FunA2() { printf("I'am ProgramA_FunA2 and be called..\n"); }

void programB_FunB1(void (*callback)()) {
  printf("I'am programB_FunB1 and be called..\n");
  callback();
}

int main(int argc, char **argv) {
  programA_FunA1();

  programB_FunB1(programA_FunA2);
}

//类的静态函数作为回调函数，但是静态函数只能访问静态的成员变量和函数，严重限制了回调函数的功能和实现
#include <iostream>

class ProgramA {
 public:
  void FunA1() { printf("I'am ProgramA.FunA1() and be called..\n"); }

  static void FunA2() { printf("I'am ProgramA.FunA2() and be called..\n"); }
};

class ProgramB {
 public:
  void FunB1(void (*callback)()) {
    printf("I'am ProgramB.FunB1() and be called..\n");
    callback();
  }
};

int main(int argc, char **argv) {
  ProgramA PA;
  PA.FunA1();

  ProgramB PB;
  PB.FunB1(ProgramA::FunA2);
}

//类的非静态函数作为回调函数，这个做法是可取的，但是每次在另外一个类下调用回调函数对的时候都需要事先知道回调函数所属的类，这个操作十分麻烦
#include <iostream>

class ProgramA {
 public:
  void FunA1() { printf("I'am ProgramA.FunA1() and be called..\n"); }

  void FunA2() { printf("I'am ProgramA.FunA2() and be called..\n"); }
};

class ProgramB {
 public:
  void FunB1(void (ProgramA::*callback)(), void *context) {
    printf("I'am ProgramB.FunB1() and be called..\n");
    ((ProgramA *)context->*callback)();
  }
};

int main(int argc, char **argv) {
  ProgramA PA;
  PA.FunA1();

  ProgramB PB;
  PB.FunB1(&ProgramA::FunA2, &PA);  // 此处都要加&
}

//将非静态的回调函数封装成静态函数，推荐的做法！这使得进程更加独立
#include <iostream>

class ProgramA {
 public:
  void FunA1() { printf("I'am ProgramA.FunA1() and be called..\n"); }

  void FunA2() { printf("I'am ProgramA.FunA2() and be called..\n"); }

  static void FunA2Wrapper(void *context) {
    printf("I'am ProgramA.FunA2Wrapper() and be called..\n");
    ((ProgramA *)context)->FunA2();  // 此处调用的FunA2()是context的函数, 不是this->FunA2()
  }
};

class ProgramB {
 public:
  void FunB1(void (ProgramA::*callback)(), void *context) {
    printf("I'am ProgramB.FunB1() and be called..\n");
    ((ProgramA *)context->*callback)();
  }

  void FunB2(void (*callback)(void *), void *context) {
    printf("I'am ProgramB.FunB2() and be called..\n");
    callback(context);
  }
};

int main(int argc, char **argv) {
  ProgramA PA;
  PA.FunA1();

  ProgramB PB;
  PB.FunB1(&ProgramA::FunA2, &PA);  // 此处都要加&

  printf("\n");
  PB.FunB2(ProgramA::FunA2Wrapper, &PA);
}

//使用std::funtion以及std::bind。std::function是一种通用、多态的函数封装，std::function的实例可以对任何可以调用的目标实体进行存储、复制、和调用操作，这些目标实体包括普通函数、Lambda表达式、函数指针、以及其它函数对象等。std::bind()函数的意义就像它的函数名一样，是用来绑定函数调用的某些参数的。
#include <iostream>

#include <functional> // fucntion/bind

class ProgramA {
 public:
  void FunA1() { printf("I'am ProgramA.FunA1() and be called..\n"); }

  void FunA2() { printf("I'am ProgramA.FunA2() and be called..\n"); }

  static void FunA3() { printf("I'am ProgramA.FunA3() and be called..\n"); }
};

class ProgramB {
  typedef std::function<void ()> CallbackFun;
 public:
   void FunB1(CallbackFun callback) {
    printf("I'am ProgramB.FunB2() and be called..\n");
    callback();
  }
};

void normFun() { printf("I'am normFun() and be called..\n"); }

int main(int argc, char **argv) {
  ProgramA PA;
  PA.FunA1();

  printf("\n");
  ProgramB PB;
  PB.FunB1(normFun);
  printf("\n");
  PB.FunB1(ProgramA::FunA3);
  printf("\n");
  PB.FunB1(std::bind(&ProgramA::FunA2, &PA));
}
```

---

## typedef

typedef是C++中用于给默认数据类型创建别名的一种关键字，它的具体用法如下：

```cpp
char *pa, *pb;
//char* pa,pb;

typedef char* pCHAR;
pCHAR pa,pb;
//这里可以发现使用typedef可以很好地解决定义多个指针变量时会出现漏写*号的问题hhh是不是很有用

struct Teacher
{   
    int age;
}Tea;  //Tea是一个变量  
​
typedef struct Student
{   
    int age;
}Stu;  //Stu是一个结构体类型 = struct Student

void main()
{
    Tea.age = 30;  //为结构成员赋值
    Stu Zhang;   //先声明结构类型变量
    Zhang.age = 15;   //访问结构成员并赋值
}
//typedef也可以给结构体等定义别名，至于作用看上面这段代码肯定能够明白
```

---

## 智能指针

在C++内存管理中专门给在程序运行时分配的对象分配了一个存储空间（堆），我们可以通过new和delete来管理堆，也就是说这些对象的内存需要程序员主动去开辟同时也需要程序员主动去释放。那么，如果我们忘了释放内存，那么内存泄露引发的一系列问题就会接踵而至，再者，如果我们释放了还在被指针指向的内存，就会出现非法访问的指针，为了更加安全方便地管理动态内存，C++提供了智能指针，这就是智能指针产生的原因。

智能指针的行为和常规指针大同小异，区别在于智能指针会自动释放掉所指向的对象，这就规避了人为操作管理内存时可能会产生的问题。

C++标准库提供了三种不同的智能指针。

### std::unique_ptr

一种独享资源所有权的指针。

```cpp
//在脱离了unique_ptr作用域后会自动释放内存
int* p = new int(100);
delete p;  
//正常指针的使用，需要手动释放内存

std::unique_ptr<int> uptr = std::make_unique<int>(200);
//...
//离开uptr的作用域的时候自动释放内存

std::unique_ptr<int> uptr = std::make_unique<int>(200);
std::unique_ptr<int> uptr1 = uptr;//这种操作是不允许的，uptr只能转移，无法通过编译
std::unique_ptr<int> uptr2 = std::move(uptr);//uptr允许转移
assert(uptr == nullptr);//这个时候uptr变成了空指针，已经完成了转移

```

### std::shared_ptr

一种共享资源所有权的指针。

```cpp
//shared_ptr是一种会对资源引用做计数的指针，当引用数为0时自动释放内存
{
    std::shared_ptr<int> sptr = std::make_shared<int>(200);
    assert(sptr.use_count() == 1);  // 此时引用计数为 1
    {   
        std::shared_ptr<int> sptr1 = sptr;
        assert(sptr.get() == sptr1.get());
        assert(sptr.use_count() == 2);   // sptr 和 sptr1 共享资源，引用计数为 2
    }   
    assert(sptr.use_count() == 1);   // sptr1 已经释放
}
// use_count 为 0 时自动释放内存
```

shared_ptr占用的内存空间要比普通指针和unique_ptr更大，这是基于它的实现原理制定的，因为shared_ptr本质上是两个指针（一个指向共享资源的指针，一个指向引用数等共享资源的控制信息，可以理解为一个管理控制信息的指针），这也是为什么自定义deleter不会影响shared_ptr的大小（因为shared_ptr的deleter是保存在控制信息上的）。

### std::weak_ptr

一种共享资源的观察者，需要和std::shared_ptr一起使用，不会影响共享资源内存的释放和开辟。

```cpp
//wptr会在sptr释放之后自动指向null，wptr也可以升级为sptr（前提是sptr没有被释放），wptr可以用来观察对象是否存活
void Observe(std::weak_ptr<int> wptr) {
    if (auto sptr = wptr.lock()) {
        std::cout << "value: " << *sptr << std::endl;
    } else {
        std::cout << "wptr lock fail" << std::endl;
    }
}

std::weak_ptr<int> wptr;
{
    auto sptr = std::make_shared<int>(111);
    wptr = sptr;
    Observe(wptr);  // sptr 指向的资源没被释放，wptr 可以成功提升为 shared_ptr
}
Observe(wptr);  // sptr 指向的资源已被释放，wptr 无法提升为 shared_ptr
```

---

## unit8_t,unit16_t,unit32_t,unit64_t

首先，C++的数据类型分为三种：整型、浮点型、布尔型。下面这张图片显示了C++基本的数据类型。

![](https://pic2.zhimg.com/v2-265f199c0c8227f3e37e31fdde879761_r.jpg)

那么unit8_t,unit16_t,unit32_t,unit64_t是什么？

这些都不是新的数据类型，而是通过typedef给类型起的别名，*_t是typedef定义的标志，是一种表示规范。

```cpp
using namespace std;

int main() {
  uint8_t a = 65;
  cout << a << endl;
  //输出unit8_t类型的变量实际输出的是对应字符，不是真实数据
  
   uint8_t a = 65;

   // uint8_t -> string
   string str;
   ostringstream oss;
   oss << a;
   str = oss.str();
   cout << str << endl;

   // string -> uint8_t
   str = "65";
   stringstream ss;
   ss >> a;
   ss.clear();
   cout << a << endl;

   uint32_t b = 66;

   // uint32_t -> string
   string str2;
   ostringstream oss2;
   oss2 << b;
   str = oss2.str();
   cout << str2 << endl;

   // string -> uint32_t
   str = "66";
   stringstream ss2;
   ss2 << str;
   ss2 >> b;
   ss2.clear();
   cout << b << endl;
   //uint8_t类型变量转化为字符串时得到的会是ASCII码对应的字符, 字符串转化为 uint8_t 变量时, 会将字符串的第一个字符赋值给变量
   //上面输出依次为：A，6，66，66 
}
```

---

## pair及其在vector中使用

pair本质上是一种类模板，它的作用是将一对值组合成一个值，而且这一对值可以是不同的数据类型。

同样，pair也支持默认的三种构造方式。

```cpp
pair<int ,double> p1;//默认构造函数

pair<int ,double> p2(1, 2.4);//用给定值初始化

pair<int ,double> p3(p2);//拷贝构造函数
```

还可以将pair放在vector中使用。

```cpp
//声明vector
vector<pair<int,int> >vec;

//用make_pair向vector中插入数据
vec.push_back(make_pair(20,30)); 
vec.push_back(make_pair<int,int>(10,50));

//定义迭代器
 vector<pair<int,int> > ::iterator iter;
for(iter=vec.begin();iter!=vec.end();iter++);

//获取数据
(*iter).first; //第一个数据
(*iter).second; //第二个数据
```

---

## boost库

### boost::function

boost::function是一个函数对象的容器，以对象的形式封装了原始的函数指针或者函数对象，能够容纳任意符合函数签名的可调用对象。在声明functions时候，函数签名是最重要的部分，因为函数签名将决定functions保存的函数或函数对象的签名和返回类型。

boost::functions能够代替指针函数，而且能够接收函数或者函数对象，大大增加了程序灵活性，但也会加大负担。

下面这个例子应该能很好地帮助理解：

```cpp
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

typedef boost::function<int(int,char)>Func;

int test(int num,char ch)
{
    std::cout<<num<<" "<<ch<<std::endl;
}

int main()
{
    Func f;
    f = &test;
    f(1,'a');
    return 0;
}

//结果
1，a
```

### boost::bind

bind的基本形式如下：

```cpp
template<class R,class F> bind(F f);
template<class R,class F,class A1> bind(F f,A1 a1);
namespace
{
boost::arg<1> _1;
boost::arg<2> _2;
boost::arg<3> _3;
…..                                     //其他6个占位符
};
```

**bind**接收的**第一个参数**必须是一个**可调用的对象f**，包括**函数**、**函数指针**、**函数对象**、和**成员函数指针**，之后bind**最多接受9个参数**，**参数数量**必须与**f的参数数量相等**，这些参数被传递给f作为入参。 绑定完成后，**bind**会**返回**一个**函数对象**，它内部**保存了f的拷贝**，具有**operator()**，**返回值类型**被**自动推导**为**f的返回类型**。在发生调用时这个**函数对象**将把之前**存储的参数**转发给**f**完成调用。

下面看个例子：

```cpp
//有一个函数func
func(a1,a2);

//它等价于一个具有无参operator()的bind函数对象调用
bind(func,a1,a2)();
```

这只是bind最简单的形式，bind表达式存储了func和a1、a2的拷贝，产生了一个临时函数对象。func接收两个参数，而a1和a2的拷贝传给了func完成真正的函数调用。

**bind**的真正威力在于它的**占位符**，它们分别定义为**_1,_2,_3**,**一直到 _9**,位于一个匿名的名字空间。**占位符**可以取代bind**参数的位置**，在发生**调用时**才**接受真正的参数**。**占位符的名字**表示它在**调用式**中的**顺序**，而在绑定的表达式中没有没有顺序的要求，_1不一定必须第一个出现，也不一定只出现一次，例如：

```cpp
bind(func,_2,_1)(a1,a2);

//返回一个具有两个参数的函数对象，第一个参数将放在func的第二个位置，而第二个参数则放在第一个位置，调用时等价
func(a2,a1);

//具体例子
#include <functional>
#include <iostream>
#include <string>
#include "boost/bind.hpp"

void print_string(const std::string s) 
{  std::cout << s << '\n';
}
void print_functionname()
{
    std::cout << "Print_functionname" <<std::endl;
}

int main()
{
    boost::bind(print_functionname)();
    return 0;
}
```

### bind和functions一起使用

```cpp
#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>

typedef boost::function<void(void)>Func;

void print_string(const std::string s)
{
    std::cout<<"s:"<<s<<std::endl;
}

int main()
{
    Func f(boost::bind(print_string,"hello bind"));
    f();
    return 0;
}
```

---

## 强制类型转换

C++中存在强制类型转换，static_cast、const_cast、rinterpret_cast以及dynamic_cast。

### static_cast

1.基本类型之间转换，不适用于基本类型指针转换，但是void指针和其他类型指针可以

```cpp
double d=0;
int i=static_cast<int>(d);
```

2.子类和父类之间的指针或引用之间转换

3.空类型指针转换为基本类型指针

```cpp
int i=0;
void *vp=&i;
int *p=static_cast<int*>(vp);
*p=3;
    cout<<i<<endl;
//这里i的初始类型为int 然后我们将i的地址转换为void*，然后void指针vp转换为int指针，此时int型指针 p和vp指向的i的原始类型int一致，所以转换结果为3

int  i=3;
void *vp=&i;
char *p=static_cast<char*>(vp);
*p=4
cout<<i<<endl;
//转换结果为4，虽然结果正确，但是这种转换方法是不安全的，最后转换的类型与数据的初始类型是不一样的，原因就在于不同的数据类型占用的字节数是不一样的，在我的机器里char是一个字节也就是8位，int是4字节，也就是32位，在第一个例子里3的二进制是0...0 0000 0011，那么在转换为char指针时实际指向低8位数据空间，也就是00000011所在位置，然后经过*p=4以后，变为00000100，因此数据i的二进制就变成了0...0 0000 0100  ，也就是4
```

---

## memset

<string.h>中封装的一个API，函数原型如下：

```cpp
void *memset(void *s,int c,unsigned long n);
//为指针变量s所指的前n个字节的内存单元填充给定的int型数值c，它可以为任何数据进行初始化。就是将数值c以单个字节逐个拷贝的方式放到指针变量s所指的内存中去

//例子
#include <iostream>
#include <string.h>
using namespace std;

int main()
{
    int dp[3];
    memset(dp,-1,sizeof(dp));

    for(int i=0;i<3;i++)
        cout << dp[i] << " ";

    return 0;
}
//因为-1在计算机中存储为：1111 1111，故dp数组中每一个int值为“1111 1111 1111 1111 1111 1111 1111 1111”，是十进制下的-1。输出结果：-1 -1 -1
```

---



