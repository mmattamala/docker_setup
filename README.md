# docker_setup
Personal docker setup with all the tools and packages I use

## General overview

* `bin`: stores all the scripts to build the images and run the containers, as well as the default settings.
* `scripts`: helper files to install the dependencies of the images.
* `targets`: scripts to load all the environment variables to setup different target options (cpu, gpu).
* `Dockerfile`: the main Dockerfile that builds the images.
* `entrypoint.sh`: the entrypoint script that is executed when the container is executed. It sources the `.bashrc` file and it can also run other stuff.

## Dependencies
This package assumes you have Docker ([link](https://docs.docker.com/engine/install/ubuntu/)) and NVidia docker ([link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)) installed.

Please also check the [Troubleshooting](#troubleshooting) section below for more common errors I had while using Docker (like enabling support for NVidia GPUs).

## Using the repo

### Setting up the image targets
The targets define different configurations we can build images for, i.e, CPU or GPU-based platforms. For example, the script [`gpu.sh`](targets/gpu.sh) configures all the variables required for my laptop.

If you want to create images for other GPUs or platforms, you can use [these files](/home/matias/git/docker_setup/targets) as a reference.


### Building the images
While staying in the `docker_setup` folder, we can build the images by setting the corresponding `target`:

```sh
./bin/build.sh --target=cpu
```

### Running a container

To run a container from an image we built, we need to set the `target` corresponding to the image, as well as the directories in the host system to the `git` and `catkin_ws` folders.

```sh
./bin/run.sh --target=cpu --git-dir=$HOME/git --catkin-dir=$HOME/catkin_ws
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