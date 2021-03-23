## Build image with cuda and opencv based on cuda

FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y && \
    apt-get install -y \
        build-essential \
        git \
        libgtk2.0-dev \
        pkg-config \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        python3 \
        python3-pip \
        wget \
        unzip \
        libssl-dev \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-tools \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        ca-certificates \
        wget \
        curl \
        checkinstall \
        autoconf \
        automake \
        libtool \
        g++ \
        yasm \
        libgoogle-glog-dev \
        libatlas-base-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        python-sphinx


## Install Eigen includes
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip && unzip eigen-3.3.9.zip && mv eigen-3.3.9/Eigen/ /usr/local/include/ && mv eigen-3.3.9/unsupported/ /usr/local/include/


## Install newest version of cmake
WORKDIR /tmp

RUN version=3.16 && \
    build=5 && \
    wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz && \
    tar -xzvf cmake-$version.$build.tar.gz && \
    cd cmake-$version.$build/ && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install


## Install Ceres
RUN     git clone https://ceres-solver.googlesource.com/ceres-solver \
        &&  cd ceres-solver \
        &&  mkdir release \
        &&  cd release \
        &&  cmake .. \
        &&  make -j3 \
        &&  make test \
        &&  make install


## Install OpenCV
WORKDIR /libs

RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    mkdir opencv/build

WORKDIR /libs/opencv/build

RUN cmake .. \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DWITH_CUDA=OFF \
        -DCUDA_ARCH_BIN=7.5 \
        -DCUDA_ARCH_PTX= \
        -DENABLE_FAST_MATH=ON \
        -DCUDA_FAST_MATH=OFF \
        -DWITH_CUBLAS=OFF \
        -DWITH_CUDNN=OFF \
        -DWITH_LIBV4L=ON \
        -DWITH_V4L=ON \
        -DWITH_GSTREAMER=ON \
        -DWITH_GSTREAMER_0_10=OFF \
        -DWITH_QT=OFF \
        -DWITH_OPENGL=ON \
        -DOPENCV_DNN_CUDA=OFF \
        -DBUILD_opencv_python2=ON \
        -DBUILD_opencv_python3=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DINSTALL_C_EXAMPLES=OFF \
        -DINSTALL_PYTHON_EXAMPLES=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DWITH_IPP=OFF \
        -DWITH_OPENCL=OFF \
        -DWITH_FFMPEG=OFF \
        -DWITH_AVFOUNDATION=OFF

RUN make -j$(nproc) && make install && ln -s /usr/include/opencv4/opencv2/ /usr/local/include/


## Finally, let's do some cleaning
WORKDIR /
RUN rm -rf libs/
RUN rm -rf eigen-3.3.9*
RUN rm -rf tmp/