# Ceres库

## 1.什么是ceres

Ceres Solver是google开源的一个C++库，用于建模以及解决大型复杂的优化问题。常用来解决具有边界约束和一般无约束优化问题的非线性最小二乘问题。

在很多大型的项目上都能看到Ceres Solver的身影，例如谷歌地图、Tango项目、注明的SLAM系统OKVIS等......

## 2.安装ceres

在ubuntu上安装ceres库：

```shell
# 使用二进制方式安装ceres需要的依赖
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev

# 开始搭建编译并安装ceres
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install

# 以上摘自Ceres Solver官方文档
```

目前ceres已知的支持ubuntu20.04和18.04版本，对于更早的版本的支持不会做维护也不保证。

## 3.ceres从入门到入土

使用ceres库来求解有界约束的非线性最小二乘问题的形式：

![](https://img-blog.csdnimg.cn/20190624161952806.png)

这种类型的问题通常出现在从拟合统计曲线和从计算机视觉中照片构建3D模型（这样子的问题在科学和工程领域很常见）。

其中，整个求和符号后面的部分也就是ρ(·)称为核函数，官方叫法`LossFunction`（损失函数），这个函数一般为恒等函数，它可以针对不同部分对误差权重进行调整也就是减少异常值对非线性最小二乘问题求解的影响。ρ(·)里面的fi函数f*()称为`CostFunction`（代价函数），![](https://img-blog.csdnimg.cn/2019062416271054.png)

称为`ParameterBlock`（参数块），我们会把需要优化的量放在这里头，参数块的规模一般很小。下面的s.t.意思是subject to （受限于），也就是参数的优化所在范围。当s.t（受限）的条件变成正无穷到负无穷时，就是我们常说的非线性最小二乘问题。

*先看从一个博客文章里看到的使用ceres求解问题的模板*

```cpp
//ceres求解步骤
//定义CostFuntion
//构建Problem
//配置Solver
//定义Summary
//开始优化Solve
//输出结果SUmmary.BriefReport 
 
 
#include <ceres/ceres.h>
using namespace std；
 
struct CostFunction{
  template <typedef T> bool operator()(const T* x, T* residual )  const{   
    residual[0]=cost_function; 
    return true;
  }           //这个步骤是必须的，通过函数重载运算操作符定义代价函数，就是形式里面的f(x)。
  
  //x的维度是下面的dim_2 
  //residual的维度是dim_1
 
  //可选
  //下面这部分，如果结构中需要传递别的数据，可以采取定义数据，然后采用才C++11标准中结构
  //体使用初始化列表初始化数据
  /*
   const data_type  _a, _b;  
  CostFunction(data_type a,data_type b): _a ( a ), _b ( b ) { } 
  */
}; 
 
 
int main( int argc ,char** argv)
{
	 
  /*
   函数其他部分,初始化 
   带估计量x的初始化 
   */
  //构建问题
  Problem problem;的优化问题。常用来解决具有边界约束和一般无约束优化问题的非线性最小二乘问题。

在很多大型的项目上都能看到Ceres Solver的身影，例如谷歌地图、Tango项目、注明的SLAM系统OKVIS等......

2.安装ceres
  //添加误差项
  problem.AddResidualBlock(
    new DiffCostFunction_type<CostFunction,dim_1,dim_2>(new CostFunction), 
    nullptr,
    &x
	);
	// DiffCostFunction_type一般可以使用自动求导 AutoDiffCostFunction
	//或者使用NumericDiffCostFunction
	
//使用时为了区别，定义结构体名为 NumericDiffCostFunction，其他相似，构建问题参数多了一个 
//    new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, dim_1,dim_2>(
//      new NumericDiffCostFunctor)，
 
  //第一个参数为生成的costfuntion，尖括号中为误差类型，输入维度，输出维度
  //dim_2 的维度是上面结构体定义的x维度 
  
  //第二个参数为核函数
  //第三个参数为待估计参数的地址，如果是数组，则可以直接传入数组首地址 
  //说明：这个x是main函数初始化部分定义的 
  
  //配置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;//线性方程求解的方法
  options.minimizer_progress_to_stdout =true;//输出优化的过程
  //优化信息结果
  ceres::Solver::Summary summary;
  //开始优化
  ceres::Solve(options, &problem, &problem );
  //输出优化结果简报
    cout<<summary.BriefReport()<<endl;
}
```

*接下来再看官方文档给出的hello world示例*

```cpp
//先看完模板再看这个应该会好理解很多
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = 10.0 - x[0];
     return true;
   }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}

```

