#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2025 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.


# This script builds the Docker exec image for the RVC

# Usage: ./docker_build_img.sh [ROS_DISTRO] [--no-cache]
# Example: ./docker_build_img.sh
# Example: ./docker_build_img.sh humble
# Example: ./docker_build_img.sh humble --no-cache

set -e

# Parse arguments
ROS_DISTRO=${1:-humble}
UBUNTU_VERSION=22.04
NO_CACHE=""

# Check for --no-cache flag in any position
for arg in "$@"; do
    if [[ "$arg" == "--no-cache" ]]; then
        NO_CACHE="--no-cache"
    fi
done

echo "Building RVC image with ROS_DISTRO=${ROS_DISTRO} and UBUNTU_VERSION=${UBUNTU_VERSION}"

# Build the Docker image
docker build $NO_CACHE \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
    -t rvc-exec:${ROS_DISTRO} \
    -f Dockerfile.exec .

if [[ $? -ne 0 ]]; then
    echo "Docker build failed."
    exit 1
fi

echo "Successfully built rvc-exec:${ROS_DISTRO}"
