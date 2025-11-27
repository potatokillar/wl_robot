1、本代码为智研院内部机器人仿真平台库，请勿外传。
2、本代码为ROS-noetic版本，请勿使用其他ROS版本。
3、本代码为C++编写，请勿使用其他语言。
4、使用方法如下：
  1）使用git clone命令获取代码；
  2）在simulation目录下使用脚本执行命令：./build.sh；
  3）在simulation目录下使用脚本执行命令：./start.sh $1 $2 $3；
    $1 为机器人类型:qr为四足机器人，arm为机械臂，human为人形机器人;
    $2 为机器人型号:qr下目前有：go1、sz、mz、sz_w、a1，arm下目前有:iris,human下有:gh1、gr1t1;
    $3 为调试模式：user为关闭，debug为开启；

5、Docker调试方法：
  使用以下命令启动容器，便于调试：
```shell
sudo docker run -v /home/wl/workspace/simulation_all:/home/wl/workspace/simulation_all -w /home/wl/workspace/simulation_all --env-file env.tmp -v /root/.ros:/root/.ros -v /root/.gazebo:/root/.gazebo --gpus=all -it --rm --network=host --privileged registry.cn-hangzhou.aliyuncs.com/iiri/build_x86_arm_ros1 bash --
```