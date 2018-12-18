ARG IMAGE_SOURCE
FROM $IMAGE_SOURCE

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive

COPY .gitlab-ci/apt-ros.org.key .
RUN apt-get update -qq && apt-get upgrade -qqy && \
    apt-get install -qqy --no-install-recommends dirmngr gnupg2 lsb-release && \
    (apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 || apt-key add apt-ros.org.key) && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get -qq update

# image comparison & virtual display support
RUN apt-get install -qqy --no-install-recommends imagemagick sudo git python-rosdep && \
    apt-get install -qqy xvfb scrot && \
    rosdep init

WORKDIR /home/dev

# Install packages and their dependencies
COPY *.deb ./
RUN (dpkg -i *.deb || true) && apt-get install -qqy -f

# Install workspace packages
COPY .gitlab-ci/sweetie_bot_flexbe_behaviors  ./src/sweetie_bot_flexbe_behaviors
COPY .gitlab-ci/sweetie_bot_proto2_movements  ./src/sweetie_bot_proto2_movements
COPY .gitlab-ci/sweetie_bot_sounds            ./src/sweetie_bot_sounds

# Create regular user "dev"
RUN export uid=1000 gid=1000 && \
    echo "dev:x:${uid}:${gid}:dev,,,:/home/dev:/bin/bash" >> /etc/passwd && \
    echo "dev:x:${uid}:" >> /etc/group && \
    echo "dev ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/dev && \
    chmod 0440 /etc/sudoers.d/dev && \
    chown ${uid}:${gid} -R /home/dev

USER dev

RUN mkdir src/thirdparty && \
    git clone https://github.com/lucasw/rviz_textured_quads.git src/thirdparty/rviz_textured_quads && \
    git clone https://github.com/FlexBE/flexbe_app.git src/thirdparty/flexbe_app

RUN rosdep update


# Build workspace
RUN bash -c 'set -e;\
    source /opt/ros/sweetie_bot/setup.bash;\
    catkin_make;\
'

# Automated testing
COPY .gitlab-ci/test.bash .gitlab-ci/reference.png ./
RUN ( ./test.bash; echo $? > ./retcode )


# For manual testing:
# 1. Uncomment section below

#RUN apt-get install -qqy openssh-server xauth && \
#    echo "AddressFamily inet" >> /etc/ssh/sshd_config
#EXPOSE 22
#ENTRYPOINT exec bash -c 'sudo /etc/init.d/ssh start; source devel/setup.bash; roslaunch sweetie_bot_deploy load_param.launch'

# 2. After container launch, execute bash into docker container and change password of user 'dev':
#    docker exec -it container sudo passwd dev
# 3. Connect via SSH to docker container with X11 forwarding enabled and launch:
#    source devel/setup.bash; roslaunch sweetie_bot_deploy flexbe_control.launch run_flexbe:=true
