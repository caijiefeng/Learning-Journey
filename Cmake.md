# **Cmake**
Cmake是一种过程式语言,可用于跨平台安装（编译）,可以产生标准的*makefile*或者*project*文件,其中***CmakeLists.txt***称为Cmake的组态档（在这里面写Cmake代码）

## **CmakeLists.txt**
指定Cmake的最小版本:
*camke_minimum_required(VERSION xxx)*
设置项目名称:
*project(name)*
生成可执行目标target:
*add_executable(executableFile main.cpp OtherFile.cpp)*
//括号内的名字自己来起
指定C++的版本:
*set(CMAKE_CXX_STANDARD_REQUIRED 14)*
指定强制使用设定的C++版本:
*set(CMAKE_CXX_STANDARD_REQUIRED ON)*
让编译器不再忽略警告:
*add_definitions(-Wall -Werror)*
*add_executable(example main.cpp OtherFile.cpp)*
添加libanswer库目标:
*add_library(libanswer STATIC OtherFile.cpp)*
指定项目的最终执行程序及文件:
*add_executable(${PROJECT_NAME} main.cpp)*
添加上层依赖:
*add_dependencies(${PROJECT_NAME} libanswer)*
添加可执行目标被链接库:
*target_link_librarise(${PROJECT_NAME} (关键字) libanswer)*
//关键字有三种.默认下为PUBLIC,PUBLIC表示上层下层目标都可以调用最初的依赖库,PRIVATE表示只有下层目标可以调用最初的依赖库,INTERFACE表示只有上层目标可以调用最初的依赖库
添加子目录:
*add_subdirectory(Folder)*
//Folder表示你的子目录名称,不添加编译时就无法进入子目录内部,也就无法执行其中的代码
对指定目标指定其依赖库C++版本:
*target_compile_features(libanswer PRIVATE cxx_std_11)*
定义变量:
*set(libanswer_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)*
打印变量:
*MESSAGE(STATUS "First Message:${libanswer_INCLUDE_DIRS}")*
指定头文件搜索路径:
*include_directories(include)*
//其中include为头文件所存放的文件夹名
寻找系统中已经安装的第三方库:
*find_package(Boost REQUIRED)*
//Boost为库名
install命令:
install(
         DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/otherfile
         DESTINATION include
         FILES_MACHING PATTERN "*.h"
        )  

## **Cmake实际操作**

```shell
mkdir filename
cd filename
cmake ..
make  //生成最终的可执行文件
./executableFile  //执行文件
```

