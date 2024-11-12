FROM osrf/ros:humble-desktop

MAINTAINER duyongquan <quandy2020@126.com>

# install packages
RUN apt clean
RUN apt update -y && \
    apt install -y \
    build-essential \
    cmake \
    curl \
    git \
    unzip \
    vim \
    wget \
    bc \
    gdb \
    python3-pip \
    libtinyxml2-dev \
    liblua5.3-dev \
    ninja-build \
    sphinx \
    uuid-dev \
    libompl-dev \
    libasio-dev \
    libtinyxml2-6 \
    libtinyxml2-dev \
    libncurses5-dev \
    libavcodec57 \
    libavcodec-dev \
    libswscale4 \
    libswscale-dev \
    libcurl4-nss-dev \
    libpoco-dev \
    libflann-dev \
    libqhull-dev \
    libpcap0.8 \
    libpcap0.8-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    software-properties-common

# Install openbot dependencies
COPY install /tmp/install

RUN bash /tmp/install/install_bazel.sh
RUN bash /tmp/installers/install_osqp.sh

# openbot workspace
ENV OPENBOT_WS /workspace

RUN mkdir -p $OPENBOT_WS
WORKDIR $OPENBOT_WS
