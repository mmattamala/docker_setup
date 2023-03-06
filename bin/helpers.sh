#!/bin/bash
# This defines some bash helpers and functions
# author: Matias Mattamala

echo_warning()
{
    # Prints WARNING in yellow and the desired message
    echo -e "\033[4;33mWARNING:\033[m $1"
}

find_substring()
{
    if grep -q "$1" <<< "$2"; then
        echo "true"
    else
        echo "false"
    fi
}

list_from_files_in_dir()
{
    # ls the directory and remove the extension
    filenames=$(ls $1/ | grep '\.sh$')

    AVAILABLE_FILENAMES=()

    # Iterate all the files and fill list of targets without the extension
    for f in $filenames; do
        # Add to list
        AVAILABLE_FILENAMES+=("${f%.*}")
    done

    echo "${AVAILABLE_FILENAMES[@]}"
}

list_stages()
{
    echo $(list_from_files_in_dir stages)
}

list_targets()
{
    echo $(list_from_files_in_dir targets)
}

check_target_exists()
{
    valid_targets="$(list_targets)"
    # We grep the list of target to find a substring
    if [[ $(find_substring "$1" "$valid_targets") == "false" ]]; then
        echo_warning "Target [$1] does not exist. Valid targets: $valid_targets"
        exit 1
    fi
}

check_stage_exists()
{
    valid_stages="$(list_stages) all"
    # We grep the list of stages to find a substring
    if [[ $(find_substring "$1" "$valid_stages") == "false" ]]; then
        echo_warning "Stage [$1] does not exist. Valid stages: $valid_stages"
        exit 1
    fi
}

check_file_exists()
{
    if [[ -f "$1" ]]; then
        echo "true"
    else
        echo "false"
    fi
}

check_directory_exists()
{
    if [[ -d "$1" && ! -L "$1" ]]; then
        echo "true"
    else
        echo "false"
    fi
}

check_container_exists()
{
    if [[ "$(docker ps -a -q -f name=$1)" ]]; then
        echo "true"
    else
        echo "false"
    fi
}

find_previous_stage()
{
    # Based on https://stackoverflow.com/a/31405855

    prev_stage=""
    # Get list out of stages string
    stages=($(echo "$(list_stages)" | tr ' ' '\n'))

    # Iterate stages
    for s in ${stages[@]}; do
        if [[ "$s" == "$1" ]]; then
            echo $prev_stage
            break
        fi

        prev_stage=$s
    done
}

packages_installed() {
    # This returns true if a package exists
    # Adapted from https://stackoverflow.com/a/54239534
    installed=true
    for pkg in $1; do
        status="$(dpkg-query -W --showformat='${db:Status-Status}' "$pkg" 2>&1)"
        if [ ! $? = 0 ] || [ ! "$status" = installed ]; then
            installed=false
            break
        fi
    done

    # Return a message to standard output
    echo "$installed"
}

run_docker_qemu()
{
    # This function install the qemu dependencies if needed and then configures docker for emulation
    if [[ $(packages_installed "qemu binfmt-support qemu-user-static") == "false" ]]; then
        # Install qemu if required
        sudo apt update
        sudo apt install qemu binfmt-support qemu-user-static -y
    fi
    
    # Run QEMU emulation
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes > /dev/null
}

compare_architectures()
{
    # This is to ensure that the architecture of an image and the system
    # match correctly in spite of the different names
    # Note: This is hacky and it should be improved
    AMD64="x86_64 amd64"
    ARM64="aarch64 arm64"
    ARM32="aarch32 arm32v6 arm32v7"
    output="false"

    if [[ $(find_substring $1 "$AMD64") == "true" && $(find_substring $2 "$AMD64") == "true" ]]; then
        output="true"
    fi

    if [[ $(find_substring $1 "$ARM64") == "true" && $(find_substring $2 "$ARM64") == "true" ]]; then
        output="true"
    fi
    
    if [[ $(find_substring $1 "$ARM32") == "true" && $(find_substring $2 "$ARM32") == "true" ]]; then
        output="true"
    fi
    
    echo "$output"
}

make_docker_setup_service_file()
{
    service_name="docker-setup-$1"
    sudo sh -c 'echo "[Unit]
Description=Docker Setup service
After=docker.service
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=always
RestartSec=2
ExecStart=docker start '$1'

[Install]
WantedBy=multi-user.target" > /etc/systemd/system/'$service_name'.service'

    sudo chmod 644 /etc/systemd/system/$service_name.service
    sudo systemctl daemon-reload
    sudo systemctl start $service_name
    sudo systemctl enable $service_name

    echo $service_name
}

remove_docker_setup_service_file()
{
    sudo rm /etc/systemd/system/docker-setup-$1.service
    sudo systemctl daemon-reload
}

