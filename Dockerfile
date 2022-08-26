FROM ubuntu:focal

# "debconf: delaying package configuration, since apt-utils is not installed"
# ARG DEBIAN_FRONTEND=noninteractive 
# RUN apt-get update \
#     && apt-get upgrade \
#     && apt-get install -q -y --no-install-recommends apt-utils

# timezone 설정
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
            tzdata \
    && rm -rf /var/lib/apt/lists/*


#######################################
#            ROS 설치 (noetic)         #
#######################################

# 사전 준비
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# sources.list 설정
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# KEY 설정
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 환경 설정
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

# CORE 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# bootstrap tools 설치
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# BASE 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*



ARG HOME=/root
#######################################
#            개발 지원 도구 설치          #                
#######################################
RUN apt-get -qq update && apt-get install -qq --no-install-recommends \
    openssh-server \
    tmux \
    git \
    vim \
    wget \
    curl \
    nano \
    libgoogle-glog-dev \
    unzip \
    libyaml-cpp-dev \

    && rm -rf /var/lib/apt/lists/*

# libtorch 설치
WORKDIR /opt
ENV LIBTORCH_URL=https://download.pytorch.org/libtorch/lts/1.8/cu111/libtorch-cxx11-abi-shared-with-deps-1.8.2%2Bcu111.zip
RUN curl -fsSL --insecure -o libtorch.zip $LIBTORCH_URL \
    && unzip -q libtorch.zip \
    && rm libtorch.zip \
    && cp /opt/libtorch/lib/* /usr/lib \
    && cp -r /opt/libtorch/include/* /usr/include \
    && rm -r /opt/libtorch



EXPOSE 22

#######################################
#     ISAS_KAIST_K 필요라이브러리 설치    #                
#######################################
RUN apt-get -qq update && apt-get install -qq --no-install-recommends \
    ros-noetic-pcl-* \
    libeigen3-dev \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy==1.19.3 numdifftools==0.9.39 scipy==1.5.4 scikit-build==0.11.1 matplotlib



#######################################
#             CUDA 도구 설치            # 
#######################################
ENV NVARCH x86_64
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.6 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 brand=tesla,driver>=450,driver<451 brand=tesla,driver>=460,driver<461 brand=tesla,driver>=470,driver<471 brand=unknown,driver>=470,driver<471 brand=nvidia,driver>=470,driver<471 brand=nvidiartx,driver>=470,driver<471 brand=quadrortx,driver>=470,driver<471"
ENV NV_CUDA_CUDART_VERSION 11.6.55-1
ENV NV_CUDA_COMPAT_PACKAGE cuda-compat-11-6

ARG TARGETARCH

RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH}/3bf863cc.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH} /" > /etc/apt/sources.list.d/cuda.list && \
    apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.6.0

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-6=${NV_CUDA_CUDART_VERSION} \
    ${NV_CUDA_COMPAT_PACKAGE} \
    && ln -s cuda-11.6 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

#######################################
#      ISAS_KAIST_K 알고리즘 빌드        # 
#######################################

RUN echo "source /opt/ros/noetic/setup.bash" >> ${HOME}/.bashrc
RUN echo "source /root/catkin_ws/devel/local_setup.bash" >> ${HOME}/.bashrc

SHELL ["/bin/bash", "-c"]
WORKDIR ${HOME}
RUN source ${HOME}/.bashrc

RUN mkdir -p ${HOME}/catkin_ws/src/KAIST_MORIN_ISAS

COPY ./KAIST_MORIN_ISAS ${HOME}/catkin_ws/src/KAIST_MORIN_ISAS
# WORKDIR ${HOME}/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ${HOME}/catkin_ws; catkin_make'

RUN ls ${HOME}/catkin_ws/src/

RUN chmod +x ${HOME}/catkin_ws/src/KAIST_MORIN_ISAS/me_asc/scripts/me_asc_node.py
RUN chmod +x ${HOME}/catkin_ws/src/KAIST_MORIN_ISAS/mf_lte/scripts/mf_lte_node.py





#######################################
#             작업 환경 구성             # 
#######################################
USER root

RUN apt-get -qq update && apt-get install -qq --no-install-recommends pip 


WORKDIR ${HOME}
RUN mkdir .ssh
COPY ./asset/ ${HOME}
RUN echo 'source ${HOME}/entrypoint.sh' >> ${HOME}/.bashrc

RUN pip install webcolors;exit 0
RUN pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113;exit 0

ENTRYPOINT service ssh restart && bash
CMD ["/bin/bash"]