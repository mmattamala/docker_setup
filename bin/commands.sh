#!/bin/bash
# This script defines the commands used to handle the setup of docker containers
# author: Matias Mattamala

dsbash()
{
    xhost +local: > /dev/null
    if [ $# -eq 0 ]; then
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            ${DOCKER_SETUP_CONTAINER_NAME} /bin/bash
    else
        args=$@
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            ${DOCKER_SETUP_CONTAINER_NAME} /bin/bash -c "source /root/catkin_ws/devel/setup.bash && ${args}"
    fi
}

dsstatus()
{
    sudo service ${DOCKER_SETUP_SERVICE} status
}