ARG IMAGE_SOURCE
FROM $IMAGE_SOURCE

RUN rm -f *.deb
COPY *.deb ./

# Install packages and their dependencies
RUN apt-get -qq update && apt-get -qq upgrade && \
    (dpkg -i *.deb || true) && apt-get install -qq -f
