# 第三方库介绍

## 说明
### 精简代码
直接拉取的源代码，会包含文档，测试程序等等。这些可以删除，只留下有用的源码。  

### 库
可以动态库，可以静态库。   
动态库存放到/usr/local/lib下面。  

## spdlog-1.9.2
https://github.com/gabime/spdlog   
一个小型的C++日志库。   
头文件包含即可   
已精简过代码。  

## toml11-3.7.0
https://github.com/ToruNiina/toml11  
一个toml的C++11解析库。若将来代码升级到c++17，也可换一个其它的库。   
头文件包含即可  
已精简过代码。

## muduo-2.0.2
https://github.com/chenshuo/muduo   
一个小型的C++网络库，单平台Linux。  

## json-3.10.2
https://github.com/nlohmann/json   
一个热门的C++ json库

## qpOASES
https://github.com/coin-or/qpOASES  
二次规划库
目前以静态库的形式编入代码中。会尝试动态库方法编入。

## JCQP
类似qpOASES

## osqp
类似qpOASES

## inih
ini解析库

## SOEM
ethercat主机库  
