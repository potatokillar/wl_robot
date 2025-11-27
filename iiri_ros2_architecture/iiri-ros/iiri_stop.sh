#!/bin/bash

container_info=$(docker ps -a | grep iiri-ros)
if [ -n "$container_info" ]; then
    # 提取容器ID
    container_id=$(echo $container_info | awk '{print $1}')
    # 使用docker stop命令关闭容器
    docker stop $container_id
    docker rm $container_id
fi