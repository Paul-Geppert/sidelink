# Adapted from https://github.com/EttusResearch/ettus-docker/blob/master/ubuntu-uhd/Dockerfile

# ettusresearch/ubuntu-uhd:20.04

# Provides a base Ubuntu (20.04) image with latest UHD installed

FROM        ubuntu:20.04

# This will make apt-get install without question
ARG         DEBIAN_FRONTEND=noninteractive
# ARG         UHD_TAG=v3.15.0.0
ARG         MAKEWIDTH=6

# Add software-properties-common for add-apt-repository
RUN         apt-get update && \
            apt-get -y install -q software-properties-common

# Install security updates and required packages
RUN         add-apt-repository ppa:ettusresearch/uhd && \
            apt-get update && \
            apt-get -y install -q \
                build-essential \
                ccache \
                git \
                python3-dev \
                python3-pip \
                curl \
# Install UHD
                libuhd-dev \
                libuhd4.2.0 \
                uhd-host \
# Install UHD dependencies
                # libboost-all-dev \
                # libusb-1.0-0-dev \
                # libudev-dev \
                # python3-mako \
                # doxygen \
                # python3-docutils \
                # python3-requests \
                # python3-numpy \
                # dpdk \
                # libdpdk-dev \
# Sidelink dependencies
                cmake \
                libfftw3-dev \
                libmbedtls-dev \
                libboost-program-options-dev \
                libconfig++-dev \
                libsctp-dev \
                libboost-all-dev \
                libusb-1.0-0-dev \
                libudev-dev \
                libulfius-dev \
                libjansson-dev \
                libmicrohttpd-dev \
                cpufrequtils && \
            rm -rf /var/lib/apt/lists/*

# Alternatively build and install uhd yourself

# RUN         mkdir -p /usr/local/src
# RUN         git clone https://github.com/EttusResearch/uhd.git /usr/local/src/uhd
# RUN         cd /usr/local/src/uhd/ && git checkout $UHD_TAG
# RUN         mkdir -p /usr/local/src/uhd/host/build
# WORKDIR     /usr/local/src/uhd/host/build
# RUN         cmake .. -DENABLE_PYTHON3=ON -DUHD_RELEASE_MODE=release -DCMAKE_INSTALL_PREFIX=/usr
# RUN         make -j $MAKEWIDTH
# RUN         make install
RUN         uhd_images_downloader
ENV         UHD_IMAGES_DIR=/usr/share/uhd/images
WORKDIR     /

RUN         mkdir /sidelink
COPY        . /sidelink

RUN         cd /sidelink/ && \
            mkdir build && \
            cd build && \
            cmake .. && \
            make -j $MAKEWIDTH

ADD docker/start_master.sh /sidelink
ADD docker/start_client_log.sh /sidelink
ADD docker/entrypoint.sh /sidelink

WORKDIR /sidelink

ENTRYPOINT ["/sidelink/entrypoint.sh"]
