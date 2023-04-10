# docker_setup

Personal package to work with Docker with all the tools and packages I use: ROS, OpenCV with CUDA support, PyTorch.
It allows to:

1. **Build new images** that rely on some of the packages mentioned above.
2. **Run them** in an easy way.
3. **Set up containers as background process** to allow easy deployment on our robots.

The available images can be found in [DockerHub](https://hub.docker.com/u/mmattamala)

</br>

## Acknowledgments
The setup here is inspired by advice and snippets by Simone Arreghini and Jonas Frey (ETH Zurich).

</br>

## General overview

* `bin`: stores all the scripts to build the images and run the containers, as well as the default settings.
* `stages`: files that define the different stages to install all the dependencies, creating intermediate images.
* `targets`: scripts to load all the environment variables to setup different target options (cpu, gpu).
* `Dockerfile`: the main Dockerfile that builds the images.
* `entrypoint.sh`: the entrypoint script that is executed when the container is executed. It sources the `.bashrc` file and it can also run other stuff.

</br>

## Dependencies

This package assumes you have installed:

* Docker >20.10.10: [link](https://docs.docker.com/engine/install/ubuntu/)
* Docker buildx: [link](https://github.com/docker/buildx#linux-packages)
* NVidia docker: [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

Additionally, if you are working on a Jetson board, you need:

* Jetson Stats: [link](https://github.com/rbonghi/jetson_stats)

Please also check the [Troubleshooting](#troubleshooting) section below for more common errors I had while using Docker (like enabling support for NVidia GPUs).

</br>

## Building approach

This repo follows the approach of manually creating different images to incrementally add more dependencies. This can be also addressed by [multi-stage builds](https://docs.docker.com/build/building/multi-stage/#use-a-previous-stage-as-a-new-stage) but I decided to do it manually to have more control on the workflow.

</br>

## Using the repo

### Setting up the image targets

The targets define different configurations we can build images for, i.e, CPU or GPU-based platforms. For example, the script [`gpu.sh`](targets/gpu.sh) configures all the variables required for my laptop.

If you want to create images for other GPUs or platforms, you can use [these files](targets) as a reference.

</br>

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

</br>

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

</br>

## Background installation

The scripts in this package also allow to have a container running as a background process, so it can be easily used to test GPU applications or launch processes via [Procman](https://github.com/ori-drs/procman_ros).
The following instructions explain how to to this in a clean computer (e.g a Jetson board).

</br>

### Pre-installation

1. Make the `git` folder if it doesn't exist `mkdir $HOME/git && cd $HOME/dir`
2. Clone this repo: `git clone git@github.com:mmattamala/docker_setup.git`
3. Create a new entrypoint for the container under `entrypoints/your_platform_name>.sh`. You can find some examples in the [`entrypoints`](entrypoints/) folder

The entrypoint is important because it allows us to launch processes at boot. Check the section below for a detailed explanation of the [`jetson_xavier_cerberus.sh`](entrypoints/jetson_xavier_cerberus.sh) example:

<details>
  <summary> Click for full explanation </summary>

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
        echo "Warning: procman_ros is not build within the catkin_ws. Therefore the deputy cannot be started!"
    else
        source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  && rosrun procman_ros deputy -i anymal_cerberus_xavier
    fi
else
    echo "Warning: Catkin_ws does not exist!"
fi
```

This entrypoint sources the `.bashrc`, `catkin_ws` and automatically starts a `procman deputy` with the correct name set to `anymal_cerberus_xavier` if `procman_ros` is correctly build within the catkin_ws. Make sure that the entrypoint can be executed otherwise the service can not run and the container will not start.

> 💡 Running the idealing procman deputy inside the container does not take up any resources, therefore it can always run in the background, even when not used.
</details>
</br>

### Installation of the container

Simply execute:

```sh
./bin/install.sh --name=jetson_xavier_cerberus --target=jetson_xavier --git=$HOME/git --entrypoint=jetson_xavier_cerberus.sh
```

The previous script will do as follows:

1. It will pull the image specified by the target. In this case the target is `jetson_xavier`, which is defined in the [targets](targets) folder: [jetson_xavier](targets/jetson_xavier.sh)
2. A new container will be created under the name `jetson_xavier_cerberus`. This is the name that will be used by the Docker daemon, and it will mount the `$HOME/git` folder under `/root/git` in the container. This container is initialized using the [`scripts/run.sh`](bin/run.sh) script described before.
3. A background service is added under: `/etc/systemd/system/docker-setup-jetson_xavier_cerberus.service`. This will ensure that the container is launched on startup, and it will execute what is defined within the entrypoint. 
4. Finally, the installation script also defines some variables and extra files that are sourced in the `bashrc` of the host.
5. Make sure to source your `.bashrc`

<details>
  <summary>Click for full explanation</summary>

```sh
# == Docker setup ini ==
export DOCKER_SETUP_ROOT=/home/tutuna/git/docker_setup
export DOCKER_SETUP_CONTAINER_NAME=jetson_xavier_cerberus
source $DOCKER_SETUP_ROOT/bin/commands.sh
export DOCKER_SETUP_SERVICE=docker-setup-jetson_xavier_cerberus
# == Docker setup end =="
```

</details>

</br>

### Post-installation

#### Sanity checks:

After sourcing your `.bashrc`, you can execute some basic tests to confirm that everything works as intended:

1. Run `docker ps` to see a running container with the name `jetson_xavier_cerberus`

```sh
user@hostname:~/git/docker_setup$ docker ps
CONTAINER ID   IMAGE                                                           COMMAND                 CREATED      STATUS         PORTS     NAMES
aece32b86eb5   mmattamala/devel-jetson:ubuntu20.04-noetic-cuda10.2.0-r32.5.0   "/entrypoint.sh bash"   1 min ago   Up 15 months             jetson_xavier_cerberus
```

2. Run `dsstatus` to check the status of the background service. It should look like:

```sh
● docker-setup-gpu.service - Docker Setup service
     Loaded: loaded (/etc/systemd/system/docker-setup-gpu.service; enabled; vendor preset: enabled)
     Active: activating (auto-restart) since Mon 2023-04-10 16:07:42 BST; 1s ago
    Process: 73193 ExecStart=/usr/bin/docker start gpu (code=exited, status=0/SUCCESS)
   Main PID: 73193 (code=exited, status=0/SUCCESS)
```

3. Run `dsbash` to start a bash terminal on the background container.

```sh
bash: /root/catkin_ws/devel/setup.bash: No such file or directory
(docker) root@hostname:~# 
```

> 💡 Both `dstatus` and `dsbash` are defined in [`bin/commands.sh`](`bin/commands.sh`), which is sourced in your `bashrc`.

> ⚠️ The error `bash: /root/catkin_ws/devel/setup.bash: No such file or directory` is expected as the `catkin_ws` has not been set up yet.

</br>

#### Setting up your catkin_ws:

1. Outside the container: clone in the `$HOME/git` folder all the repos/packages ou need. They will be automatically mounted to the container.
2. Open a new terminal on the container with `dsbash`
3. Set up the catkin workspace as usual. Here we show an example with `procman_ros`:

```sh
mkdir -p ~/catkin_ws/src/
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ln -s /root/git/procman_ros /root/catkin_ws/src/

cd $HOME/catkin_ws
catkin build procman_ros
```

4. Now log out of the container using `exit` or _Ctrl+D_ and restart the container `docker restart jetson_xavier_cerberus`. This will ensure that the procman deputy is loaded with the correct packages available within the `catkin_ws`.

> 💡 By defining `catkin_ws` inside the container we avoid conflicts between the host user and container's `root` user.

</br>

#### Running code:

<details>
  <summary> Launching code within the container </summary>

  The simplest way is just running:

  ```sh
  dsbash <your_command>
  ```

  `dsbash` should pass your command to the container to execut it.

  Otherwise, you can init a terminal on the container:
  
  ```sh
  dsbash
  ```

  And then run your command as usual:

  ```sh
  <your_command>
  ```

</details>

</br>

<details>
  <summary> Launching code on the robot </summary>
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
</details>

</br>

<details>
  <summary> Launching code with procman (work in progress) </summary>

> ⚠️ This is not fully documented yet

1. Define on OPC a procman configuration `~/procman.pmd` containing the following:

```sh
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

> ⚠️ If you change things in `catkin_ws` and you want to start them via the procman, remember to restart the container such that `catkin_ws` is sourced when procman restarts.

</details>

</br>

## Troubleshooting

Just writing down some we have found working with Docker in different platforms and how to fix them.

</br>

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

</br>

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

</br>

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

</br>

### If you get `unknown flag: --build-arg` when building images

You are missing [Docker buildx](https://github.com/docker/buildx#linux-packages). Follow the instructions so set it up in your system.

</br>

### Mount extra devices when running the container (e.g cameras)

You can use the `--flags` parameter. For example, to load a camera with descriptor `/dev/video2` as `video0` in the container:

```sh
./bin/run.sh --target=cpu --flags="-v=/dev/video2:/dev/video0"
```
