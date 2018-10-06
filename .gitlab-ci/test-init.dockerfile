ARG IMAGE_SOURCE="ubuntu:xenial"
FROM $IMAGE_SOURCE

COPY .gitlab-ci/apt-ros.org.key .
RUN apt-get update -qq && apt-get upgrade -qq && \
    apt-get install -qq --no-install-recommends dirmngr gnupg2 lsb-release && \
    (apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 || apt-key add apt-ros.org.key) && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get -qq update

# image comparison & virtual display support
RUN apt-get install -qq --no-install-recommends imagemagick sudo git python-rosdep && \
    apt-get install -qq xvfb scrot && \
    rosdep init

WORKDIR /home/dev
COPY *.deb ./

# Install packages and their dependencies
RUN (dpkg -i *.deb || true) && apt-get install -qq -f
