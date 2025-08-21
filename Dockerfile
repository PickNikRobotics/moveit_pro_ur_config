# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
ARG MOVEIT_STUDIO_BASE_IMAGE=picknikciuser/moveit-studio:${STUDIO_DOCKER_TAG:-main}
ARG USERNAME=studio-user
ARG USER_UID=1000
ARG USER_GID=1000

##################################################
# Starting from the specified MoveIt Pro release #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} AS base

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Set real time limits
# Ensure the directory exists
RUN mkdir -p /etc/security

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    cp -r /etc/skel/. /home/${USERNAME} && \
    mkdir -p \
      /home/${USERNAME}/.ccache \
      /home/${USERNAME}/.config \
      /home/${USERNAME}/.ignition \
      /home/${USERNAME}/.colcon \
      /home/${USERNAME}/.ros && \
    chown -R $USER_UID:$USER_GID /home/${USERNAME} /opt/overlay_ws/

# Add user to dialout group to enable communication with serial USB devices (gripper, FTS, ...)
# Add user to video group to enable communication with cameras
RUN usermod -aG dialout,video ${USERNAME}

# Add user to the realtime group to enable RT limits
RUN groupadd realtime && \
    usermod -a -G realtime ${USERNAME}

# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
# NOTE: The /opt/overlay_ws folder contains MoveIt Pro binary packages and the source file.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,target=${USER_WS}/,source=. \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src

# Set up colcon defaults for the new user
USER ${USERNAME}
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
COPY colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml

# hadolint ignore=DL3002
USER root

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base AS user-overlay-dev

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano \
	tmux

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]


##################################################
# Starting from the specified MoveIt Pro release with CUDA GPU #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} AS base-gpu

# CUDA repo keyring (official)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends wget ca-certificates gnupg && \
    wget -qO /tmp/cuda-keyring.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm -f /tmp/cuda-keyring.deb && \
    apt-get update && \
    # CUDA 12.8 user-space libs (no driver!)
    apt-get install -y --no-install-recommends \
      cuda-cudart-12-8 \
      libcublas-12-8 \
      libcurand-12-8 \
      libcufft-12-8 \
      # Add minimal CUDA CLI so nvcc/version is detectable by installers
      cuda-nvcc-12-8 \
      cuda-command-line-tools-12-8 \
      # cuDNN 9 for CUDA 12
      libcudnn9-cuda-12 \
      libcudnn9-dev-cuda-12 \
      # TensorRT 10.9 runtime (and parsers/plugins)
      libnvinfer10 \
      libnvinfer-plugin10 \
      libnvonnxparsers10 \
      # ZED SDK runtime deps
      libusb-1.0-0 \
      udev \
      libx11-6 \
      libxrandr2 \
      libxinerama1 \
      libxcursor1 \
      libxxf86vm1 \
      libgl1 \
      libopengl0 \
      libv4l-0 \
      v4l-utils \
      zstd \
      libgomp1 \
      pciutils \
      software-properties-common && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Ensure canonical CUDA symlink exists for installers that look at /usr/local/cuda
RUN if [ ! -e /usr/local/cuda ] && [ -d /usr/local/cuda-12.8 ]; then ln -s /usr/local/cuda-12.8 /usr/local/cuda; fi

# Optional: ONNX Runtime GPU (x86-64 only)
RUN if [ "$(uname -m)" = "x86_64" ]; then pip3 install --no-cache-dir onnxruntime_gpu==1.19.0; fi

# Copy and install ZED SDK 5.0.5 (CUDA 12.8 + TensorRT 10.9)
COPY installers/ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.5.zstd.run /tmp/zed_sdk.run
RUN chmod +x /tmp/zed_sdk.run && \
    /bin/bash /tmp/zed_sdk.run -- silent skip_tools && \
    rm -f /tmp/zed_sdk.run

# ZED + CUDA env
ENV ZED_SDK_ROOT=/usr/local/zed
ENV ZED_DIR=$ZED_SDK_ROOT
ENV CMAKE_PREFIX_PATH=$ZED_SDK_ROOT:$ZED_SDK_ROOT/share:$CMAKE_PREFIX_PATH
# DEBUG
RUN printf '%s\n' "/usr/local/cuda-12.8/targets/x86_64-linux/lib" "$ZED_SDK_ROOT/lib" > /etc/ld.so.conf.d/zed.conf && ldconfig
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=$ZED_SDK_ROOT/bin:$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$ZED_SDK_ROOT/lib:/usr/local/cuda-12.8/targets/x86_64-linux/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Create non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
# Idempotent user/group creation that reuses existing GID/UID if they already exist
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -euo pipefail; \
    apt-get update; \
    apt-get install -q -y --no-install-recommends sudo; \
    # Resolve (or create) group with desired GID
    if getent group "$USER_GID" >/dev/null; then \
      GROUP_NAME="$(getent group "$USER_GID" | cut -d: -f1)"; \
    else \
      # If the *name* exists but with a different GID, don't fail; just create by GID
      if getent group "$USERNAME" >/dev/null; then \
        GROUP_NAME="$USERNAME"; \
      else \
        groupadd --gid "$USER_GID" "$USERNAME"; \
        GROUP_NAME="$USERNAME"; \
      fi; \
    fi; \
    # Resolve (or create) user with desired UID
    if getent passwd "$USER_UID" >/dev/null; then \
      EXISTING_USER="$(getent passwd "$USER_UID" | cut -d: -f1)"; \
      # If the username differs, rename it to $USERNAME (best-effort)
      if [ "$EXISTING_USER" != "$USERNAME" ]; then usermod -l "$USERNAME" "$EXISTING_USER" || true; fi; \
      usermod -g "$USER_GID" -s /bin/bash "$USERNAME" || true; \
      HOME_DIR="$(getent passwd "$USERNAME" | cut -d: -f6)"; \
      [ -d "$HOME_DIR" ] || mkdir -p "$HOME_DIR"; \
    else \
      useradd --uid "$USER_UID" --gid "$USER_GID" --shell /bin/bash --create-home "$USERNAME"; \
      HOME_DIR="/home/$USERNAME"; \
    fi; \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME"; \
    chmod 0440 "/etc/sudoers.d/$USERNAME"; \
    mkdir -p "$HOME_DIR/.ccache" "$HOME_DIR/.config" "$HOME_DIR/.ignition" "$HOME_DIR/.colcon" "$HOME_DIR/.ros"; \
    chown -R "$USER_UID:$USER_GID" "$HOME_DIR" "/opt/overlay_ws/"


# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
# NOTE: The /opt/overlay_ws folder contains MoveIt Pro binary packages and the source file.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,target=${USER_WS}/,source=. \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src

# Set up colcon defaults for the new user
USER ${USERNAME}
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
COPY colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml

# hadolint ignore=DL3002
USER root

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base-gpu  AS user-overlay-gpu-dev

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]

#########################################
# Target for compiled, deployable image with GPU support #
#########################################
FROM base-gpu AS user-overlay-gpu

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/onnxruntime/capi:/usr/lib/x86_64-linux-gnu:/usr/local/cuda-12.6/targets/x86_64-linux/lib:$LD_LIBRARY_PATH

# Compile the workspace
WORKDIR $USER_WS

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]


#########################################
# Target for compiled, deployable image #
#########################################
FROM base AS user-overlay

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Compile the workspace
WORKDIR $USER_WS

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]
