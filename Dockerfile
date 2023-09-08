FROM osrf/ros:noetic-desktop-full


# Environment variable -> set language to C (computer) UTF-8 (8 bit unicode transformation format).
ENV LANG C.UTF-8

# Debconf is used to perform system-wide configutarions.
# Noninteractive -> use default settings -> put in debconf db.
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime.
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Environment variable -> see output in real time.
ENV PYTHONUNBUFFERED 1

# Install some handy tools.
RUN set -x \
        && apt-get update \
        && apt-get upgrade -y \
        && apt-get install -y  terminator  python3-catkin-tools \
	&& apt install snapd -y

# The OSRF container didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python
COPY ./Projects/ $HOME/user/Projects/
WORKDIR $HOME/user/Projects/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && apt-get install -y \
    python3-pip
RUN wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
RUN python3 -m pip install depthai


RUN apt install ros-noetic-depthai* -y
