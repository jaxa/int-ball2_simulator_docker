# ステージ1: Qtをインストール
FROM --platform=linux/amd64 ubuntu:18.04 as qt-builder

# Qtインストールに必要な依存関係をインストール
RUN apt update && apt install -y wget

# Qtインストーラをダウンロード
RUN mkdir -p /opt/Qt/install
RUN cd /opt/Qt && wget https://download.qt.io/archive/qt/5.12/5.12.3/qt-opensource-linux-x64-5.12.3.run

# 外部で生成したqsファイルをコピー
COPY qt-installer-nonintaractive.qs /opt/Qt/

# Qtをインストール
RUN cd /opt/Qt && \
    chmod +x qt-opensource-linux-x64-5.12.3.run && \
    chmod +x qt-installer-nonintaractive.qs && \
    ./qt-opensource-linux-x64-5.12.3.run --script qt-installer-nonintaractive.qs --verbose --platform minimal && \
    ln -s /opt/Qt/install/5.12.3 /opt/Qt/5

# ステージ2: 最終イメージ
FROM --platform=linux/amd64 ubuntu:18.04

RUN apt clean && apt update &&\
    apt install -y wget git vim curl gnupg2 lsb-release iproute2

# Install Python3 packages
RUN apt install -y python3 python3-pip
RUN pip3 install rospkg 
RUN pip3 install empy==3.3.4
RUN pip3 install psutil

# Time zone settings
RUN DEBIAN_FRONTEND=noninteractive apt install -y tzdata
ENV TZ=Asia/Tokyo

# Install ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt -y install ros-melodic-desktop-full
RUN apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install gazebo packages
RUN apt install -y ros-melodic-gazebo-*

RUN sed -i '/ROS_PYTHON_VERSION/s/^/#/; /ROS_PYTHON_VERSION/a export ROS_PYTHON_VERSION=3' /opt/ros/melodic/etc/catkin/profile.d/1.ros_python_version.sh

# Install Netwide Assembler (NASM)
RUN cd /usr/local/src &&\
    wget https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/nasm-2.15.05.tar.gz &&\
    tar zxvf nasm-2.15.05.tar.gz &&\
    cd nasm-2.15.05 &&\
    ./configure &&\
    make install

# Install Video Reception Environment
RUN git clone https://code.videolan.org/videolan/x264.git /usr/local/src/x264-master &&\
    cd /usr/local/src/x264-master &&\
    ./configure --disable-asm --enable-shared --enable-static --enable-pic &&\
    make install

RUN git clone https://github.com/FFmpeg/FFmpeg.git -b n4.1.3 /usr/local/src/ffmpeg-4.1.3 &&\
    cd /usr/local/src/ffmpeg-4.1.3 &&\
    ./configure --extra-cflags="-I/usr/local/include" \
                --extra-ldflags="-L/usr/local/lib" \
                --extra-libs="-lpthread -lm -ldl -lpng" \
                --enable-pic \
                --disable-programs \
                --enable-shared \
                --enable-gpl \
                --enable-libx264 \
                --enable-encoder=png \
                --enable-version3 &&\
    make install

# Install VLC Media Player
RUN apt install -y libasound2-dev libxcb-shm0-dev libxcb-xv0-dev \
                libxcb-keysyms1-dev libxcb-randr0-dev libxcb-composite0-dev \
                lua5.2 lua5.2-dev protobuf-compiler bison libdvbpsi-dev libpulse-dev
RUN cd /usr/local/src &&\
    wget https://get.videolan.org/vlc/3.0.7.1/vlc-3.0.7.1.tar.xz &&\
    tar Jxvf vlc-3.0.7.1.tar.xz &&\
    cd /usr/local/src/vlc-3.0.7.1 &&\
    cp ./share/vlc.appdata.xml.in.in ./share/vlc.appdata.xml &&\
    CFLAGS="-I/usr/local/include" \
    LDFLAGS="-L/usr/local/lib" \
    X264_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    X264_LIBS="-lx264" \
    X26410b_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    X26410b_LIBS="-lx264" \
    AVCODEC_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    AVCODEC_LIBS="-lavformat -lavcodec -lavutil" \
    AVFORMAT_CFLAGS="-L/usr/local/lib -I/usr/local/include" \
    AVFORMAT_LIBS="-lavformat -lavcodec -lavutil" \
    ./configure --disable-a52 \
                --enable-merge-ffmpeg \
                --enable-x264 \
                --enable-x26410b \
                --enable-dvbpsi &&\
    make install &&\
    ln -s /usr/local/src/vlc-3.0.7.1 /usr/local/src/vlc

# Qtインストール済みディレクトリをコピー（認証情報を含まない）
COPY --from=qt-builder /opt/Qt /opt/Qt

# Install Font File
RUN apt install -y fonts-roboto

# Install Docker
RUN apt update &&\
    apt install -y init systemd
RUN apt install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN apt update
RUN apt install -y docker-ce docker-ce-cli containerd.io
RUN pip3 install docker defusedxml netifaces

# Download and build int-ball2_simulator
WORKDIR /home 
RUN git clone https://github.com/jaxa/int-ball2_simulator.git
RUN cd /home/int-ball2_simulator/Int-Ball2_platform_gse &&\
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make"
RUN apt install -y libpcl-dev ros-melodic-pcl-ros &&\
    cd /home/int-ball2_simulator/Int-Ball2_platform_simulator &&\
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make –DWITH_PCA9685=OFF"

# Download int-ball2_platform_works repository
WORKDIR /home 
RUN git clone https://github.com/jaxa/int-ball2_platform_works.git

# Docker-in-Dockerは実行時に設定
CMD ["/bin/bash"]