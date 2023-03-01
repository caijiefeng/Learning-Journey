# clion

## install

https://www.jetbrains.com/clion/download/#section=linux

clion官方下载通道，clion官方提供了适用于Windows、macOS以及Linux的版本，选择Linux版本的下载。

下载后的压缩包一般会在/dowanload目录下，双击解压或者使用命令行解压：

```shell
cd /dowanlad
tar -xzf CLion-2021.1.3.tar.gz
```

一般这样解压后的clion会出现在/dowanload目录下，这样子使用运行时clion会出现崩溃现象，按照官方教程应该将clion解压在/opt目录下：

```shell
cd /opt
mkdir clion
sudo mv clion-2021.1.3/ /opt/clion/
#具体根据下载的clion版本号
```

这样子就完成了clion的安装！

## user

推荐使用命令行打开clion：

```shell
sh /opt/clion/clion-2021.1.3/bin/clion.sh
```

到这里就可以开始使用clion了。

## development

相关配置，下面根据dynamicx队内要求进行配置。



