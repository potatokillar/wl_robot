#! /bin/bash
sudo docker run -d --rm -it \
                -p 1883:1883 \
                -p 8083:8083 \
                -p 8084:8084 \
                -p 8883:8883 \
                -p 18083:18083 \
                docker.1ms.run//emqx/emqx