ARG image
FROM $image

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive

COPY .gitlab-ci/apt-ros.org.key .
RUN apt-get update -qq && apt-get upgrade -qqy \
 && apt-get install -qqy --no-install-recommends dirmngr gnupg2 lsb-release \
 && (apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 || apt-key add apt-ros.org.key) \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
 && apt-get -qq update

# Install packages and their dependencies
COPY *.deb ./
RUN dpkg -i *.deb 2>/dev/null; apt-get install -qqy -f && rm *.deb

# image comparison & virtual display support & user
RUN apt-get install -qqy --no-install-recommends imagemagick sudo git python-rosdep xvfb scrot \
 && rosdep init \
 && useradd -ms /bin/bash dev && mkdir -p /home/dev/src && chown dev:dev -R /home/dev \
 && echo "dev ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/dev \
 && chmod 0440 /etc/sudoers.d/dev

USER dev
WORKDIR /home/dev

# Install workspace packages
RUN mkdir src/thirdparty \
 && git clone https://github.com/lucasw/rviz_textured_quads.git src/thirdparty/rviz_textured_quads \
 && git clone https://github.com/FlexBE/flexbe_app.git src/thirdparty/flexbe_app \
 && rosdep update
COPY --chown=dev:dev .gitlab-ci/sweetie_bot_flexbe_behaviors  ./src/sweetie_bot_flexbe_behaviors
COPY --chown=dev:dev .gitlab-ci/sweetie_bot_proto2_movements  ./src/sweetie_bot_proto2_movements
COPY --chown=dev:dev .gitlab-ci/sweetie_bot_sounds            ./src/sweetie_bot_sounds

# Build workspace
RUN bash -c 'set -e;\
    source /opt/ros/sweetie_bot/setup.bash;\
    catkin_make;\
'

# Automated testing
COPY .gitlab-ci/test.bash .gitlab-ci/reference.png ./
RUN ./test.bash; echo $? > ./retcode
