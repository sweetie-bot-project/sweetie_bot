#!/bin/bash
set -e

if [[ -z "$FTP_SERV" ]]; then
	echo 'FTP_SERV environment variable is expected to be set as package output path' >&2
	echo 'You may `export` it in ~/.profile file or `export` it explicitly' >&2
	exit 2
fi

print_usage() {
	echo "Usage: docker-build.bash <stretch-armhf|stretch|xenial>" >&2
}

SUFFIX=""
case "$1" in
	stretch-armhf)
		SUFFIX="-armhf"
		DISTRO="stretch"
		;;
	stretch)
		DISTRO="stretch"
		;;
	xenial)
		DISTRO="xenial"
		;;
	*)
		print_usage
		exit 1
		;;
esac

DEPENDS="docker.io qemu-user-static binfmt-support"
dpkg -l $DEPENDS >/dev/null 2>/dev/null || sudo apt-get install --no-install-recommends $DEPENDS

sudo docker pull "slavanap/compile_orocos:lunar-$DISTRO$SUFFIX"

# cleanup of artifacts from previous build
sudo docker rm "temp-sweetie-$DISTRO$SUFFIX" || true
sudo docker rmi "sweetie-image-$DISTRO$SUFFIX" || true

# gen dockerfile and run build
BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOCKERFILE="$BASH_DIR/lunar-$DISTRO$SUFFIX.dockerfile"
cat >"$DOCKERFILE" <<-EOF
FROM slavanap/compile_orocos:lunar-$DISTRO$SUFFIX
COPY . /tmp/repo/sweetie_bot/src
RUN ./travis-armhf.bash sweetie-bot-pkg
EOF

sudo docker build -t "sweetie-image-$DISTRO$SUFFIX" -f "$DOCKERFILE" "$BASH_DIR"
sudo docker create --name "temp-sweetie-$DISTRO$SUFFIX" "sweetie-image-$DISTRO$SUFFIX"
sudo docker cp "temp-sweetie-$DISTRO$SUFFIX:/tmp/repo/sweetie-bot$SUFFIX.deb" "$FTP_SERV/sweetie-bot$SUFFIX~$DISTRO.deb"

# cleanup
rm "$DOCKERFILE"
