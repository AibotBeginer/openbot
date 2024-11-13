FROM osrf/ros:humble-desktop

MAINTAINER duyongquan <quandy2020@126.com>

# # install packages
# RUN apt clean
# RUN apt update -y && \
#     apt install -y \
#     build-essential \
#     cmake \
#     curl \
#     git \
#     unzip \
#     vim \
#     wget \
#     bc \
#     gdb \
#     libtinyxml2-dev \
#     liblua5.3-dev \
#     ninja-build \
#     sphinx \
#     python3-sphinx \
#     uuid-dev \
#     libompl-dev \
#     libasio-dev \
#     libtinyxml2-dev \
#     libncurses5-dev \
#     libavcodec-dev \
#     libswscale-dev \
#     libcurl4-nss-dev \
#     libpoco-dev \
#     libflann-dev \
#     libqhull-dev \
#     libpcap0.8 \
#     libpcap0.8-dev \
#     libusb-1.0-0 \
#     libusb-1.0-0-dev \
#     software-properties-common

# Setup entrypoint
COPY workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]

# Install openbot dependencies
COPY install /tmp/install

# thirdparty
RUN mkdir /thirdparty
# RUN bash /tmp/install/install_bazel.sh
# RUN bash /tmp/install/install_bazel_packages.sh
# RUN bash /tmp/install/install_python_modules.sh
RUN bash /tmp/install/install_cyberrt.sh
# RUN bash /tmp/install/install_osqp.sh
# RUN bash /tmp/install/install_g2o.sh
# RUN bash /tmp/install/install_ceres_solver.sh
# RUN bash /tmp/install/install_behaviortree_cpp.sh

# openbot workspace
ENV OPENBOT_WS /workspace

RUN mkdir -p $OPENBOT_WS
WORKDIR $OPENBOT_WS
