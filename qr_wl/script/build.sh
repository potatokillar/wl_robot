#!/bin/bash
# build.sh; 编译所有程序，包括x86和arm
# build.sh -c; 删除缓存并编译所有程序。
# build.sh x86; 编译x86的程序，包括sim,test
# build.sh arm; 编译arm的程序
# build.sh -c x86; 删除x86缓存并编译x86的程序
# build.sh -c arm; 删除arm缓存并编译arm的程序

# 请用scipt/build.sh来调用本脚本，不要用cd script && ./build.sh调用
ARCH="all"
if [ $# == 1 ];then
    if [ "$1" == '-c' ]; then
        echo "clean all build cache"
        rm -rf build
    fi

    if [ "$1" == 'x86' ]; then
        ARCH="x86"
    fi

    if [ "$1" == 'arm' ]; then
        ARCH="arm"
    fi
fi

if [ $# == 2 ];then
    if [ "$1" != '-c' ]; then
        echo "option error"
        exit 1
    fi

    if [ "$2" == 'x86' ]; then
        ARCH="x86"
        echo "clean x86/64 build cache"
        rm -rf build/x64
    fi

    if [ "$2" == 'arm' ]; then
        ARCH="arm"
        echo "clean arm/arm64 build cache"
        rm -rf build/arm64
    fi
fi

# 不存在则创建
if [ ! -d "./build/x64" ]; then
    mkdir -p build/x64
fi

if [ ! -d "./build/arm64" ]; then
    mkdir -p build/arm64
fi

cd build || exit

# 编译前处理
python3 ../script/prevBuild.py

# 留一个核用于其他任务
get_ncpu=$(expr $(nproc) - 1 )
# 同时编译x64和arm64的程序
if [ $ARCH == "all" ] || [ $ARCH == "x86" ]; then
    echo ">>>>>>>>>build x86/64<<<<<<<<<<"
    cd x64 || exit
    cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/toolchain_x64.cmake ../../ -DPYTHON_EXECUTABLE=/usr/bin/python3
    make -j"$get_ncpu"
    cd ..

    # 不存在则创建
    if [ ! -d "./x64/output" ]; then
        mkdir -p ./x64/output
    fi

    # 手动复制到对应目录
    # 因为频繁存在，可执行文件未被成功编译或编译成功但未复制的情况
    if [ -e "./x64/project/real/qr" ]; then
        mv ./x64/project/real/qr ./x64/output
    fi

    #if [ -e "./x64/project/sim/qr_sim" ]; then
    #    mv ./x64/project/sim/qr_sim ./x64/output
    #fi

    #if [ -e "./x64/test/qr_test" ]; then
    #   mv ./x64/test/qr_test ./x64/output
    #fi

    #if [ -e "./x64/project/vrep/qr_vrep" ]; then
    #    mv ./x64/project/vrep/qr_vrep ./x64/output
    #fi
fi

if [ ${ARCH} == "all" ] || [ ${ARCH} == "arm" ]; then
    echo ">>>>>>>>>>build arm/arm64<<<<<<<<<<"
    cd arm64 || exit
    cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake/toolchain_arm64.cmake ../../ 
    make -j"$get_ncpu"
    cd ..
    
    if [ ! -d "./arm64/output" ]; then
        mkdir -p ./arm64/output
    fi
    
    if [ -e "./arm64/project/real/qr" ]; then
        mv ./arm64/project/real/qr ./arm64/output
    fi
fi

# 编译后处理
python3 ../script/afterBuild.py