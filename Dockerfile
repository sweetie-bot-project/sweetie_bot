# Usage:
# $ sudo apt-get install --no-install-recommends docker.io qemu-user-static binfmt-support
# $ sudo docker build -t my-sweetie-image .
# $ sudo docker create --name temp-container my-sweetie-image
# $ sudo docker cp temp-container:/tmp/repo/sweetie-armhf.deb ./sweetie-armhf_0.1~stretch.deb

FROM slavanap/compile_orocos:lunar

COPY . /tmp/repo/sweetie_bot/src

RUN ./travis-armhf.bash sweetie-pkg
