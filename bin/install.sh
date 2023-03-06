#!/bin/bash
# This script setups the container as a service, so it can run at startup as a background process
# author: Matias Mattamala
set -e

# Include helpers
source bin/helpers.sh

# Define usage
__usage="
Usage: $(basename $0) --target=TARGET|--image=IMAGE [OPTIONS]

Options:
  --name=<name>            Name for the container. Default: background_container
  --target=<target>        Selected target (available ones): [$(list_targets)]
  --image=<image>          Tag or image id of a docker image (useful to launch other Docker images)

  --stage=<stage>          Use specific stage: [$(list_stages)] (It requires a target to work)
  --git=<git_folder>       Git folder to be mounted (Default: $HOME/git)
  --catkin=<ws_folder>     Catkin workspace folder to be mounted (Default: $HOME/catkin_ws)
  --entrypoint=<file>      Custom entrypoint script to be executed when running the container
  --flags=<docker_flags>   Any extra flags passed do docker run (e.g, to mount additional stuff)
"

# Default target
CONTAINER_NAME=background_container
TARGET=none
IMAGE_ID=none
STAGE=""
GIT_DIR="$HOME/git"
CATKIN_DIR="$HOME/catkin_ws"
ENTRYPOINT_FILE="dummy.sh"
EXTRA_FLAGS=""
UNINSTALL_FLAG=""

# Read arguments
for i in "$@"; do
    case $i in
        --name=*)
            CONTAINER_NAME=${i#*=}
            shift
            ;;
        --target=*)
            TARGET=${i#*=}
            shift
            ;;
        --stage=*)
            STAGE=${i#*=}
            shift
            ;;
        --image=*|--image-id=*)
            IMAGE_ID=${i#*=}
            shift
            ;;
        --git=*|--git-dir=*)
            GIT_DIR=${i#*=}
            shift
            ;;
        --catkin=*|--catkin-dir=*)
            CATKIN_DIR=${i#*=}
            shift
            ;;
        --entrypoint=*)
            ENTRYPOINT_FILE=${i#*=}
            shift
            ;;
        --flags=*)
            EXTRA_FLAGS=${i#*=}
            shift
            ;;
        --uninstall)
            UNINSTALL_FLAG="true"
            shift
            ;;
        *)
            echo "$__usage"
            exit 0
            ;;
    esac
done

# Helper strings
DS_INI="# == Docker setup ini =="
DS_END="# == Docker setup end =="

if [[ ${UNINSTALL_FLAG} != "true" ]]; then
    # Set environment variables in .bashrc
    echo "Installing Docker Setup for container [$CONTAINER_NAME]"
    echo "${DS_INI}" >> ~/.bashrc
    echo "export DOCKER_SETUP_ROOT=$(pwd)" >> ~/.bashrc
    echo "export DOCKER_SETUP_CONTAINER_NAME=${CONTAINER_NAME}" >> ~/.bashrc
    echo "source \$DOCKER_SETUP_ROOT/bin/commands.sh" >> ~/.bashrc
    
    # Temporary exports
    export DOCKER_SETUP_ROOT=$(pwd)
    export DOCKER_SETUP_CONTAINER_NAME=${CONTAINER_NAME}
    source $DOCKER_SETUP_ROOT/bin/commands.sh

    # Run the container once, with the given name
    echo "Creating container [${DOCKER_SETUP_CONTAINER_NAME}]..."
    if [[ $(check_container_exists ${DOCKER_SETUP_CONTAINER_NAME}) == "true" ]]; then
        docker container restart ${DOCKER_SETUP_CONTAINER_NAME}

    else
        # Run new container
        ${DOCKER_SETUP_ROOT}/bin/run.sh \
                            --target=$TARGET \
                            --image=$IMAGE \
                            --stage=$STAGE \
                            --git=$GIT_DIR \
                            --catkin=$CATKIN_DIR \
                            --entrypoint=$ENTRYPOINT_FILE \
                            --no-rm \
                            --flags="--name=${DOCKER_SETUP_CONTAINER_NAME} -d" > /dev/null
    fi

    # Install service
    docker container stop ${DOCKER_SETUP_CONTAINER_NAME}
    export DOCKER_SETUP_SERVICE=$(make_docker_setup_service_file ${DOCKER_SETUP_CONTAINER_NAME})
    echo $DOCKER_SETUP_SERVICE
    echo "Adding service [${DOCKER_SETUP_SERVICE}]..."
    echo "export DOCKER_SETUP_SERVICE=${DOCKER_SETUP_SERVICE}" >> ~/.bashrc
    sudo service ${DOCKER_SETUP_SERVICE} restart

    # Final message
    echo "${DS_END}" >> ~/.bashrc
    echo "Container [${DOCKER_SETUP_CONTAINER_NAME}] installed! Do not forget to source your .bashrc"

else
    echo "Uninstalling Docker Setup for container [$CONTAINER_NAME]"

    # Stop service
    echo "Stopping and removing service [${DOCKER_SETUP_SERVICE}]"
    sudo service ${DOCKER_SETUP_SERVICE} stop

    # Remove service file
    remove_docker_setup_service_file ${DOCKER_SETUP_CONTAINER_NAME}

    # Stop container
    echo "Stopping and removing container [${DOCKER_SETUP_CONTAINER_NAME}]"
    docker container stop ${DOCKER_SETUP_CONTAINER_NAME}

    # Remove container
    docker container rm ${DOCKER_SETUP_CONTAINER_NAME}

    # Remove entries from .bashrc
    sed -i "/${DS_INI}/,/${DS_END}:/d" ~/.bashrc

    # Unset variables
    unset DOCKER_SETUP_ROOT
    unset DOCKER_SETUP_CONTAINER_NAME
    unset DOCKER_SETUP_SERVICE
fi

