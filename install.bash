#!/bin/bash
set -e

echo "Installing essential dependencies..."
apt-get update -qq
apt-get install -qqy --no-install-recommends dirmngr gnupg2 lsb-release wget

if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
	echo "Enabling ROS repository..."
	apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
fi

if [[ ! -f /etc/apt/sources.list.d/sweetie-bot.list ]]; then
	echo "Enabling SweetieBot repository..."
	wget -qO - https://sweetie-bot.gitlab.io/sweetie_bot/repository.key | apt-key add -
	echo "deb http://sweetie-bot.gitlab.io/sweetie_bot $(lsb_release -sc) main" > /etc/apt/sources.list.d/sweetie-bot.list
fi

echo "Refreshing package cache..."
apt-get update -qq

echo
echo "Now you're able to install SweetieBot packages. For instance, try to type:"
echo
echo "sudo apt-get install ros-lunar-sweetie-bot"
echo
echo
