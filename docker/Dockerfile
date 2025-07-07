ARG UBUNTU_DISTRO=20.04
ARG ANY_RELEASE=release-23.05
##############
# Base image #
##############
FROM anybotics-docker:${ANY_RELEASE}-ubuntu${UBUNTU_DISTRO}-base as base

ARG ROS_DISTRO=noetic
ARG CATKIN_WORKSPACE_DIR="/catkin_ws"

WORKDIR ${CATKIN_WORKSPACE_DIR}
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Realsense Prerequisites
RUN apt-get update \
    && apt-get install -y \
    ros-noetic-realsense2-description \
    ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Unitree Prerequisites
RUN apt-get update \
    && apt-get install -y \
    libboost-dev \
    libeigen3-dev \
    pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

###################
# Developer image #
###################
FROM base as dev

ARG USERNAME="developer"
ARG UID=1000
ARG GID=1000

RUN apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && addgroup --gid ${GID} ${USERNAME} \
    && adduser --disabled-password --gecos '' --uid ${GID} --gid ${GID} ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME} \
    && chown -R ${UID}:${GID} ${CATKIN_WORKSPACE_DIR}

COPY "/.vscode" "${CATKIN_WORKSPACE_DIR}/.vscode"
RUN sed -i -E "s/(ROS_DISTRO=\\$\{1:-\").*(\"\})/\1${ROS_DISTRO}\2/" ${CATKIN_WORKSPACE_DIR}/.vscode/ros_source.bash \
    && echo "alias rsource='source ${CATKIN_WORKSPACE_DIR}/.vscode/ros_source.bash'" >> /home/${USERNAME}/.bash_aliases \
    && echo "alias rbuild='(cd ${CATKIN_WORKSPACE_DIR} && catkin build)'" >> /home/${USERNAME}/.bash_aliases \
    && echo "alias rclean='(cd ${CATKIN_WORKSPACE_DIR} && catkin clean -y)'" >> /home/${USERNAME}/.bash_aliases \
    && echo "alias real_mode=\"export ROS_MASTER_URI=http://${ROBOT_IP}:11311; export ROS_IP=${YOUR_IP}\"" >> /home/${USERNAME}/.bashrc \
    && echo "alias sim_mode='unset ROS_MASTER_URI; unset ROS_IP; export ROS_MASTER_URI=http://localhost:11311'" >> /home/${USERNAME}/.bashrc \
    && echo "source ${CATKIN_WORKSPACE_DIR}/.vscode/ros_source.bash" >> /home/${USERNAME}/.bashrc


# ---------------------------------------------------------------

# install nano for editing within docker if needed
RUN apt-get update && apt-get install nano -y

# install orion packages: tmux and openni2-camera package
RUN apt-get update \
 && apt-get install -y \
    python3-pip \
    tmux \
    openni2-utils \
    portaudio19-dev \
    python3-pyaudio \
    python3-sphinx \
 && rm -rf /var/lib/apt/lists/*
# RUN pip install open3d
RUN pip install numpy

RUN python3 -m pip install --upgrade pip setuptools wheel
RUN pip install numba --upgrade
RUN pip install requests --upgrade

# Manipulation dependencies
RUN pip install imutils
RUN apt-get update && apt install ros-noetic-octomap 
RUN apt-get update && apt install ros-noetic-octomap-server -y 
RUN apt-get update && apt install ros-noetic-octomap-ros -y
RUN apt-get update && apt install ros-noetic-rviz-visual-tools -y

# Navigation dependencies
RUN pip install transform3d

# Perception dependencies
RUN pip install einops
RUN pip install seaborn

RUN pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116

# Use login shell to read variables from `~/.profile` (to pass dynamic created variables between RUN commands)
SHELL ["sh", "-lc"]

# The following `ARG` are mainly used to specify the versions explicitly & directly in this docker file, and not meant
# to be used as arguments for docker build (so far).

ARG PYTORCH='2.1.0'
# (not always a valid torch version)
ARG INTEL_TORCH_EXT='2.1.0'
# Example: `cu102`, `cu113`, etc.
ARG CUDA='cu118'

RUN apt update && apt install -y git libsndfile1-dev tesseract-ocr espeak-ng python3 python3-pip ffmpeg git-lfs
RUN git lfs install
RUN python3 -m pip install --no-cache-dir --upgrade pip

# TODO: Handle these in a python utility script
RUN [ ${#PYTORCH} -gt 0 -a "$PYTORCH" != "pre" ] && VERSION='torch=='$PYTORCH'.*' ||  VERSION='torch'; echo "export VERSION='$VERSION'" >> ~/.profile
RUN echo torch=$VERSION

RUN [ "$PYTORCH" != "pre" ] && python3 -m pip install --no-cache-dir -U $VERSION torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/$CUDA || python3 -m pip install --no-cache-dir -U --pre torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/nightly/$CUDA

RUN python3 -m pip install --no-cache-dir -U tensorflow==2.13 protobuf==3.20.3 tensorflow_text tensorflow_probability

# RUN python3 -m pip install --no-cache-dir -e ./transformers[dev,onnxruntime]
RUN python3 -m pip install transformers==4.44.2

RUN python3 -m pip uninstall -y flax jax

RUN python3 -m pip install --no-cache-dir intel_extension_for_pytorch==$INTEL_TORCH_EXT -f https://developer.intel.com/ipex-whl-stable-cpu

RUN python3 -m pip install --no-cache-dir git+https://github.com/facebookresearch/detectron2.git pytesseract
RUN python3 -m pip install -U "itsdangerous<2.1.0"

# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/accelerate@main#egg=accelerate
RUN python3 -m pip install accelerate

# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/peft@main#egg=peft
RUN python3 -m pip install peft

# Add bitsandbytes for mixed int8 testing
RUN python3 -m pip install --no-cache-dir bitsandbytes

# Add auto-gptq for gtpq quantization testing
RUN python3 -m pip install --no-cache-dir auto-gptq --extra-index-url https://huggingface.github.io/autogptq-index/whl/cu118/

# Add einops for additional model testing
RUN python3 -m pip install --no-cache-dir einops

# Add autoawq for quantization testing
# RUN python3 -m pip install --no-cache-dir https://github.com/casper-hansen/AutoAWQ/releases/download/v0.1.6/autoawq-0.1.6+cu118-cp38-cp38-linux_x86_64.whl

# For bettertransformer + gptq 
# RUN python3 -m pip install --no-cache-dir git+https://github.com/huggingface/optimum@main#egg=optimum
RUN python3 -m pip install optimum
# For video model testing
# RUN python3 -m pip install --no-cache-dir decord av==9.2.0

RUN python3 -m pip install markupsafe==2.0.1
RUN python3 -m pip install trl==0.10.1
RUN python3 -m pip install jinja2==3.1.2
RUN python3 -m pip install wand
RUN python3 -m pip install ultralytics
RUN python3 -m pip install numpy==1.22

# SpeechToText
RUN apt-get update \
 && apt-get install -y \
   alsa-utils\
    pulseaudio \
    openjdk-11-jre\
     xserver-xorg-video-all\
   libgl1-mesa-glx libgl1-mesa-dri\
 && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install SpeechRecognition

# For reachability graph
RUN apt-get update \
&& apt install -y ros-noetic-exotica
RUN python3 -m pip install rtree
# TODO: rebuilt to see which open3d works (there is another installation earlier this file)
RUN python3 -m pip install open3d --ignore-installed 

RUN python3 -m pip install trimesh
RUN apt-get update && apt-get install -y libzmq3-dev
RUN python3 -m pip install --ignore-installed meshcat
RUN python3 -m pip install networkx


# Update and install ROS Noetic base
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN  apt-get update && apt install ros-noetic-move-base-msgs


ENV DEBIAN_FRONTEND=dialog

USER ${USERNAME}
