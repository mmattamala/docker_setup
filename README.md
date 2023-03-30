# docker_setup
Personal docker setup with all the tools and packages I use: ROS, OpenCV with CUDA support, PyTorch

The available images can be found in [DockerHub](https://hub.docker.com/u/mmattamala)

## Acknowledgments
The setup here is inspired by advice and snippets by Simone Arreghini and Jonas Frey (ETH Zurich).

## General overview

* `bin`: stores all the scripts to build the images and run the containers, as well as the default settings.
* `stages`: files that define the different stages to install all the dependencies, creating intermediate images.
* `targets`: scripts to load all the environment variables to setup different target options (cpu, gpu).
* `Dockerfile`: the main Dockerfile that builds the images.
* `entrypoint.sh`: the entrypoint script that is executed when the container is executed. It sources the `.bashrc` file and it can also run other stuff.

## Dependencies
This package assumes you have installed:
- Docker >20.10.10: [link](https://docs.docker.com/engine/install/ubuntu/)
- Docker buildx: [link](https://github.com/docker/buildx#linux-packages)
- NVidia docker: [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

Additionally, if you are working on a Jetson board, you need:
- Jetson Stats: [link](https://github.com/rbonghi/jetson_stats)

Please also check the [Troubleshooting](#troubleshooting) section below for more common errors I had while using Docker (like enabling support for NVidia GPUs).

## Building approach
This repo follows the approach of manually creating different images to incrementally add more dependencies. This can be also addressed by [multi-stage builds](https://docs.docker.com/build/building/multi-stage/#use-a-previous-stage-as-a-new-stage) but I decided to do it manually to have more control on the workflow.


## Using the repo

### Setting up the image targets
The targets define different configurations we can build images for, i.e, CPU or GPU-based platforms. For example, the script [`gpu.sh`](targets/gpu.sh) configures all the variables required for my laptop.

If you want to create images for other GPUs or platforms, you can use [these files](targets) as a reference.


### Building the images
While staying in the `docker_setup` folder, we can build the images using the `build.sh` script. For instance, to build all the stages of the `cpu` target:

```sh
./bin/build.sh --target=cpu
```

All the options are:
```sh
Usage: build.sh --target=TARGET [OPTIONS]

Options:
  --target=<target>        Target to be built: [cpu gpu jetson_xavier]
  --stage=<stage>          Last stage to be built: [01-base 02-cuda 03-opencv 04-ros 05-ml 06-extra]
  --no-push                DO NOT push images to DockerHub
```

### Running a container

To run a container from an image we built, we instead use the `run.sh` script. For example:

```sh
./bin/run.sh --target=cpu --git=$HOME/git
```

All the options are:
```sh
Usage: run.sh --target=TARGET|--image=IMAGE [OPTIONS]

Options:
  --target=<target>        Selected target (available ones): [cpu gpu jetson_xavier]
  --image=<image>          Tag or image id (if not using a specific target)
  --git=<git_folder>       Git folder to be mounted (Default: /home/matias/git)
```


### Pushing images to the cloud

To be completed


# Quick-Start Setting Up

## Pre-installation: 
In the following we will install the jetson xavier docker image on the robot cerberus.  
Clone the repository:
```
cd $HOME/git && git clone git@github.com:mmattamala/docker_setup.git
```

Create a new entrypoint of the container under `entrypoints/jetson_xavier_cerberus.sh`:
```sh
#!/bin/bash

# Fix for skimage
export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0

# User specific enviornment configuration
export ENV_WORKSTATION_NAME=jetson

# Assess if catkin_ws can be sourced
catkin_ws=$(echo ~/catkin_ws/devel/setup.bash)
if [ -f "$catkin_ws" ]
then
    # Assess if procman is build
    out=$(source ~/.bashrc && source ~/catkin_ws/devel/setup.bash && echo roscd procman_ros)
    ref=$(echo roscd: No such package/stack \'procman_ros\')
    if [ $out == $ref ]; then
        source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  
        echo "Warning: procman_ros is not build within the catkin_ws. Therefore the debuty cannot be started!"
    else
        source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  && rosrun procman_ros deputy -i anymal_cerberus_xavier
    fi
else
    echo "Warning: Catkin_ws does not exist!"
fi
```
This entrypoint sources the `.bashrc`, `catkin_ws` and automaticially start a `procman debuty` with the correct name set to `anymal_cerberus_xavier` if procman_ros is correctly build within the catkin_ws. Make sure that the entrypoint can be executed otherwise the service can not run and the container will not start.

As a template for the entrypoint you can use the `entrypoints/jetson_xavier_coyote.sh`  

`Sidenote:` Running the idealing procman debuty inside the container does not take up any resources, therefore it can always run in the background, even when not used.

## Installation of the container:
Simply execute:
```sh
./bin/install.sh --name=jetson_xavier_cerberus --target=jetson_xavier --git=$HOME/git --entrypoint=jetson_xavier_cerberus.sh
```

Explanation:
1. The correct docker image is pulled/build.
2. A background service is added under: `/etc/systemd/system/docker-setup-jetson_xavier_cerberus.service`
The service is starts automaticially on startup the same container and executes what is defined within the entrypoint. 
3. The following files will be appended to the `.bashrc`
```sh
# == Docker setup ini ==
export DOCKER_SETUP_ROOT=/home/tutuna/git/docker_setup
export DOCKER_SETUP_CONTAINER_NAME=jetson_xavier_cerberus
source $DOCKER_SETUP_ROOT/bin/commands.sh
export DOCKER_SETUP_SERVICE=docker-setup-jetson_xavier_cerberus
```
Sourcing the commands, setting system variables for the path to the docker_setup repository, container name and background service name.

4. Within the install script the `bin/run.sh` script is called, which directly spins up the container.
5. You should now when executing `docker ps` see a running container with the name `jetson_xavier_cerberus`

```shell
tutuna@anymal-cerberus-jetson:~/git/docker_setup$ docker ps
CONTAINER ID   IMAGE                                                           COMMAND                 CREATED      STATUS         PORTS     NAMES
aece32b86eb5   mmattamala/devel-jetson:ubuntu20.04-noetic-cuda10.2.0-r32.5.0   "/entrypoint.sh bash"   1 min ago   Up 15 months             jetson_xavier_cerberus
```
6. Make sure to source your `.bashrc`

## Setting up your catkin_ws:
1. You can now use `dsbash` to obtain a bash shell inside the container - this command is defined within `bin/commands.sh`
2. The git folder is mapped inside the container. You should pull the packages outside of the container given that no ssh keys are mapped inside.
3. Create the typicial `catkin_ws/src` and set it up as usually:
```sh
mkdir -p ~/catkin_ws/src/
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ln -s /root/git/anymal_rsl /root/catkin_ws/src/
ln -s /root/git/catkin_simple /root/catkin_ws/src/
ln -s /root/git/elevation_mapping_cupy /root/catkin_ws/src/
ln -s /root/git/glog_catkin /root/catkin_ws/src/
ln -s /root/git/procman_ros /root/catkin_ws/src/
ln -s /root/git/pybind11_catkin /root/catkin_ws/src/
ln -s /root/git/raw_image_pipeline /root/catkin_ws/src/
ln -s /root/git/grid_map /root/catkin_ws/src/
cd $HOME/catkin_ws
catkin build procman_ros
catkin build anymal_c_rsl_jetson
```
4. Having the catkin_ws not mapped inside the container avoids permission issues between the host user and container defined user. 
5. Now log out of the container using `exit` and restart the container `docker restart jetson_xavier_cerberus` - This will restart the container and ensure that procman is loaded with the correct packages available within the catkin_ws.

## Now we are ready to run the your code
### Classicial way:
1. Ssh into the jetson
```sh
ssh anymal-cerberus-jetson
```
2. Start a tmux session
```sh
tmux
```
3. Execute dsbash
```sh
dsbash
```
4. Execute the anymal_rsl code inside the container
```sh
rosrun anymal_rsl jetson.py
```
5. Done

### Procman: 
1. Define on OPC a procman configuration `~/procman.pmd` containing the following:
```
group "0.basic" {
    cmd "0.5.jetson MAN" {
        exec = "rosrun anymal_c_rsl jetson.py";
        host = "anymal_cerberus_xavier";
    }
}
```
We recommend to add an alias to the OPC `.bashrc` to easily access procman:
```sh
alias procman="rosrun procman_ros sheriff -l $HOME/procman.pmd"
```
Start procman (don not forget to source your `.bashrc` before):
```sh
procman
```


## Changing the content of the catkin_ws
Make sure when changing things inside the `catkin_ws` and you want to start them via the procman to restart the container such that procman is restarted and the catkin_ws newly sourced. 

## Troubleshooting
Just writing down some annoying problems I found and how to fix them

### To avoid using sudo
Add your user to the Docker group

Create the docker group if it does not exist
```sh
sudo groupadd docker
 ```

Add your user to the docker group.
```sh
sudo usermod -aG docker $USER
```

### Enable the nvidia runtime

Stop the Docker daemon:

```sh
sudo service docker stop
```

Open the docker daemon file
```sh
sudo nano /etc/docker/daemon.json
```

Add the `default-runtime` entry to the `daemon.json` file and save:

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

Start the daemon again
```sh
sudo service docker restart
```

### Change default directory for docker images
The images are stored in `/var/lib/docker` by default. To change it, modify the same `daemon.json` file to add the `data-root` entry:

```json
{
    "data-root": "/media/drs-orin/orin_ssd/docker",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

If you need more information when doing this on a Jetson, check this [link](https://forums.developer.nvidia.com/t/change-docker-image-storage-location-to-nvme-ssd/156882/2) from the NVidia forums.

### If you get `unknown flag: --build-arg` when building images
You are missing [Docker buildx](https://github.com/docker/buildx#linux-packages). Follow the instructions so set it up in your system.

### Mount extra devices when running the container (e.g cameras)
You can use the `--flags` parameter. For example, to load a camera with descriptor `/dev/video2` as `video0` in the container:

```sh
./bin/run.sh --target=cpu --flags="-v=/dev/video2:/dev/video0"
```