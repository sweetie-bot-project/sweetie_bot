ARG image
FROM $image

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive

COPY .gitlab-ci/apt-ros.org.key .
RUN apt-get update -qq && apt-get upgrade -qqy \
 && apt-get install -qqy --no-install-recommends dirmngr gnupg2 lsb-release \
 && (apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || apt-key add apt-ros.org.key) \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
 && apt-get -qq update

# Install packages and their dependencies
COPY *.deb ./
RUN dpkg -i *.deb 2>/dev/null; apt-get install -qqy -f && rm *.deb

# image comparison & virtual display support & user
RUN apt-get install -qqy --no-install-recommends imagemagick sudo git python3-rosdep xvfb scrot git \
 && rosdep init \
 && useradd -ms /bin/bash dev && mkdir -p /home/dev/src && chown dev:dev -R /home/dev \
 && echo "dev ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/dev \
 && chmod 0440 /etc/sudoers.d/dev

USER dev
WORKDIR /home/dev

# Install workspace packages
RUN git clone --depth=1 https://gitlab.com/sweetie-bot/sweetie_bot_flexbe_behaviors \
 && git clone --depth=1 https://gitlab.com/sweetie-bot/sweetie_bot_proto2_movements \
 && git clone --depth=1 https://gitlab.com/sweetie-bot/sweetie_bot_sounds

# Build workspace
RUN rosdep update -r || true \
 && . /opt/ros/sweetie_bot/setup.sh \
 && catkin_make

# Automated testing
COPY .gitlab-ci/test.bash .gitlab-ci/reference.png ./
RUN ./test.bash; echo $? > ./retcode
