#! /usr/bin/env bash

# This script syncs the source code from this machine onto the car.
# It uses .gitignore as a file exclusion list.
#
# Usage:
#  ./carsync CAR_IP_ADDRESS

if [[ "$1" == "" ]]; then
    echo "Please specify the car's IP address"
    exit 1
fi

rsync -am --delete --stats --filter=':- .gitignore' . amrl_user@"$1":~/"$(basename $(realpath .))"
