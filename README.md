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
./bin/run.sh --target=cpu --git=$HOME/git --catkin=$HOME/catkin_ws
```

All the options are:
```sh
Usage: run.sh --target=TARGET|--image=IMAGE [OPTIONS]

Options:
  --target=<target>        Selected target (available ones): [cpu gpu jetson_xavier]
  --image=<image>          Tag or image id (if not using a specific target)
  --git=<git_folder>       Git folder to be mounted (Default: /home/matias/git)
  --catkin=<ws_folder>     Catkin workspace folder to be mounted (Default: /home/matias/catkin_ws)
```


### Pushing images to the cloud

To be completed

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