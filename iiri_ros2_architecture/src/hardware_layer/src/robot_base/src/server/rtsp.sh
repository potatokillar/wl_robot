#! /bin/bash
# rtsp服务器启动脚本，直接启动

sudo docker run -d --rm -it \
        -e MTX_PROTOCOLS=tcp \
        -e MTX_WEBRTCADDITIONALHOSTS=127.0.0.1 \
        -p 8554:8554 \
        -p 1935:1935 \
        -p 8888:8888 \
        -p 8889:8889 \
        -p 8890:8890/udp \
        -p 8189:8189/udp \
        docker.1ms.run/bluenviron/mediamtx