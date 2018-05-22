# Usage:
# $ sudo apt-get install --no-install-recommends docker.io qemu-user-static binfmt-support
# $ sudo docker build -t my-sweetie-image .

FROM slavanap/compile_orocos

COPY . /tmp/repo/sweetie_bot/src

RUN apt-get install libalglib-dev

RUN ( source "/opt/ros/$ROS_DISTRO/setup.bash" && \
	catkin_make_isolated --install --install-space "/opt/ros/sweetie" -DCMAKE_BUILD_TYPE=Release )
