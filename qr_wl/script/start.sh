#!/bin/bash


echo $@

# 默认启动真机
if [ $# == 1 ];then
    if [ "$1" == 'qr' ]; then
        ./build/x64/output/qr ./config/qr-linkV2-3.toml
    elif [ "$1" == 'arm' ]; then
        ./build/x64/output/qr ./config/arm-iris.toml
    elif [ "$1" == 'hum' ]; then
        ./build/x64/output/qr ./config/human-mit.toml
    elif [ "$1" == 'test' ]; then
        ./build/x64/app/test/test_app
        ./build/x64/control/quadruped/test/test_qr
        ./build/x64/control/arm/test/test_arm
    fi
# 启动仿真或真机
elif [ $# == 2 ]; then
    # 四足启动
    if [ "$1" == 'qr' ]; then
        if [ "$2" == 'mz' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/qr-gazebo-mz.toml
        elif [ "$2" == 'sz' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/qr-gazebo-sz.toml
        elif [ "$2" == 'sz-w' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/qr-gazebo-sz-w.toml
        elif [ "$2" == 'test' ]; then
            ./build/x64/control/quadruped/test/test_qr
        else
            ./build/x64/output/qr ./config/qr-"$2".toml
        fi
    # 机械臂启动
    elif [ "$1" == 'arm' ]; then
        if [ "$2" == 'ros' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/arm-gazebo.toml
        elif [ "$2" == 'iris' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/arm-iris.toml
        elif [ "$2" == 'qa01' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/arm-qa01.toml
        elif [ "$2" == 'test' ]; then
            ./build/x64/control/arm/test/test_arm
        elif [ "$2" == 'ga701' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/arm-ga701.toml
        else
            ./build/x64/output/qr ./config/arm-"$2".toml
        fi 
    # 人型启动 
    elif [ "$1" == 'hum' ]; then
        if [ "$2" == 'ros' ]; then
            source ./build/x64/devel/setup.bash
            rosrun sim_bridge sim_bridge_node ./config/human-gazebo.toml
        elif [ "$2" == 'mit' ]; then
            ./build/x64/output/qr ./config/human-mit.toml
        elif [ "$2" == 'test' ]; then
            ./build/x64/app/test/test_app
        else
            ./build/x64/output/qr ./config/human-"$2".toml
        fi 
    fi
# debug启动
elif [ $# == 3 ]; then
    if [ "$3" == 'debug' ]; then
        ./build/x64/output/qr ./config/"$2".toml debug
    elif [ "$3" == 'real' ]; then
        ./build/x64/output/qr ./config/"$2".toml 
    fi
else
    echo "please input bootargs!!"
fi

