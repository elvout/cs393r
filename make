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
    # is already present in ROS_PACKAGE_PATH. Return from the function
    # if the current working directory is already present in
    # ROS_PACKAGE_PATH.
    for p in "${RPP[@]}"; do
        # `-ef` compares device and inode numbers for equality
        if [[ "$p" -ef "$CWD" ]]; then
            return
        fi
    done

    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${CWD}"
}


function use-clang() {
    # clang tends to compile the project in less time and using less
    # memory than gcc. This function tries to set the environment
    # variables used by the Makefile to the latest version of clang,
    # if it exists on the system.

    # On Ubuntu 20.04, the latest version of clang is 12.
    # I had trouble compiling the project with clang 10, but try
    # using it anyway.
    for VERSION in {12..10}; do
        if [[ -x /usr/bin/clang-$VERSION ]]; then
            export C_compiler="/usr/bin/clang-$VERSION"
            export CXX_compiler="/usr/bin/clang++-$VERSION"
            break
        fi
    done
}


if [[ $(uname -m) == "x86_64" ]]; then
    use-clang
fi
fix-ros-path
make -j
