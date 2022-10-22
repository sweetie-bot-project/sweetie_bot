#!/bin/bash
set -e

echo "Installing essential dependencies..."
apt-get update -qq
apt-get install -qqy --no-install-recommends dirmngr gnupg2 lsb-release wget

if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
	echo "Enabling ROS repository..."
	wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
	echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
fi

if [[ ! -f /etc/apt/sources.list.d/sweetie-bot.list ]]; then
	echo "Enabling SweetieBot repository..."
	wget -qO - https://apt.sweetie.bot/repository.key | apt-key add -
	echo "deb http://apt.sweetie.bot $(lsb_release -sc) main" > /etc/apt/sources.list.d/sweetie-bot.list
fi

echo "Refreshing package cache..."
apt-get update -qq

echo
echo "Now you're able to install SweetieBot packages. For instance, try to type:"
echo
echo "sudo apt-get install ros-noetic-sweetie-bot"
echo
echo
