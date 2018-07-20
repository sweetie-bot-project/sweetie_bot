# Sweetie Bot robot control system software by Sweetie Bot Project

# Build status

## Base package

Platform        | Status
----------------|--------------
Desktop         | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=master)](https://travis-ci.org/slavanap/ros-build)
Raspberry Pi 3  | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=rpi3)](https://travis-ci.org/slavanap/ros-build/branches)

## Main package

[![Build Status](https://gitlab.com/sweetie-bot/sweetie_bot/badges/devel/build.svg)](https://gitlab.com/sweetie-bot/sweetie_bot/pipelines)

# How to install binary packages

To install binary packages you need to add two repositaries:

1. Sweetie Bot Project repository
2. Official ROS repositary for dependencies

We build packages for Ubuntu 16.04, Debian 9 Stretch and Rasbian:

```
### 1. Add apt keys
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 5523BAEEB01FA116
$ wget -O - https://raw.githubusercontent.com/slavanap/ros-build/master/slavanap.key | sudo apt-key add -

### 2. Setup apt
$ sudo -i
# echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# echo "deb http://sweetie.bot/apt $(lsb_release -sc) main" > /etc/apt/sources.list.d/sweetie-bot.list

### 3. Install
$ sudo apt-get update
$ sudo apt-get install ros-lunar-sweetie-bot
```

To install from sources use Docker images from [here](https://hub.docker.com/r/slavanap/ros-build/tags/).


