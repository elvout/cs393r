#! /usr/bin/env bash

# This script dynamically inserts the assignment repository path into
# ROS_PACKAGE_PATH so that the dotfiles on each machine don't need to
# be manually updated.


function fix-ros-path() {
    # This function inserts the current working directory into the
    # ROS_PACKAGE_PATH environment variable.

    CWD=$(pwd)

    # Split ROS_PACKAGE_PATH by colons, and store the resulting tokens in
    # an array named RPP.
    if [[ "$ZSH_NAME" != "" ]]; then
        # `read` in zsh uses capital -A for some reason
        # it also numbers arrays starting with 1
        IFS=':' read -rA RPP <<< "${ROS_PACKAGE_PATH}"
    else
        IFS=':' read -ra RPP <<< "${ROS_PACKAGE_PATH}"
    fi

    # Iterate through RPP and check if the current working directory
    # is already present in ROS_PACKAGE_PATH. Return from the script if the
    # current working directory is already present in ROS_PACKAGE_PATH.
    for p in "${RPP[@]}"; do
        # `-ef` comares device and inode numbers for equality
        if [[ "$p" -ef "$CWD" ]]; then
            return
        fi
    done

    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${CWD}"
}


fix-ros-path
make -j
