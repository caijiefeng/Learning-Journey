# **Catkin**
Catkin是一个基于Cmake的编译构建系统,基本写法与Cmake大致相同
不同的是Catkin的软件包必须要有两个文件:**package.xml**,**CmakeLists.txt**

## **CmakeLists.txt**
与Cmake大致相同,但也有一些区别
catkin自带的依赖库必须要有:

*find_package(catkin REQUIRED
COMPONENTS
roscpp
std_msgs
image_geometry)*
非catkin的依赖库要单独写:
*find_package(OpenCV REQUIRED)* //OpenCV是一个系统库
指定catkin信息到编译系统:
*catkin_package(
INCLUDE_DIRS  //头文件生成的路径
include       //存放头文件的文件名
LIBRARIES     //项目导出的代码库
${PROJECT_NAME}_core   //cpp文件存放的库名
CATKIN_DEPENDS    //项目需要的Catkin依赖
roscpp
std_msgs
image_geometry
DEPENDS      //项目需要的非catkin依赖
OpenCV
)*
头文件路径:
*include_directories(
include
${catkin_INCLUDE_DIRS}
)*
生成cpp库:
*add_library(${PROJECT_NAME}_core
src/xxx.cpp
src/xxx.cpp
)*
*add_executable(${PROJECT_NAME} src/Main.cpp)*
单元测试test用法:
*if (${CATKIN_ENABLE_TESTING})
catkin_add_gtest(${PROJECT_NAME}-test
test/Test1.cpp
test/Test2.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif()*

## **package.xml**
*<package format="2">* 格式2
*<name>* 软件包名称
*<version>* 版本0.0.0
*<description>* 软件包描述
*<maintainer>* 维护人员名称
*<license>* 软件许可证
*<author email=...>* 介绍原作者
*<buildtool_depend>* 编译构建系统工具
*<depend>* 执行以及构建需要的依赖 

