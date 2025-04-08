# ステージ1: Qtをインストール
FROM --platform=linux/amd64 ubuntu:18.04 as qt-builder
# 注: --platform フラグは Qt インストーラーが x86 専用のため必要

# ビルド引数として認証情報を受け取る
# 注: ビルド時のみ使用し、最終イメージには含まれない安全な方法
ARG QT_EMAIL
ARG QT_PASSWORD
# hadolint ignore=DL3023

# Qtインストールに必要な依存関係をインストール
RUN apt-get update && apt-get install -y \
    wget \
    libdbus-1-3 \
    libfontconfig1 \
    libx11-6 \
    libx11-xcb1 \
    libxext6 \
    libxkbcommon-x11-0 \
    libxrender1 \
    libgl1-mesa-glx

# Qtインストーラをダウンロード
RUN mkdir -p /opt/Qt/install
RUN cd /opt/Qt && wget https://download.qt.io/archive/qt/5.12/5.12.3/qt-opensource-linux-x64-5.12.3.run

# インストーラースクリプトを動的に生成
RUN cd /opt/Qt && echo 'function Controller() { \
    installer.autoRejectMessageBoxes(); \
    installer.setMessageBoxAutomaticAnswer("installationError", QMessageBox.Retry); \
    installer.setMessageBoxAutomaticAnswer("installationErrorWithRetry", QMessageBox.Retry); \
    installer.setMessageBoxAutomaticAnswer("DownloadError", QMessageBox.Retry); \
    installer.setMessageBoxAutomaticAnswer("archiveDownloadError", QMessageBox.Retry); \
    installer.installationFinished.connect(function() { \
        gui.clickButton(buttons.NextButton); \
    }) \
} \
\
Controller.prototype.WelcomePageCallback = function() { \
    gui.clickButton(buttons.NextButton, 7000); \
} \
\
Controller.prototype.CredentialsPageCallback = function() { \
    var page = gui.pageWidgetByObjectName("CredentialsPage"); \
    page.loginWidget.EmailLineEdit.setText("'"$QT_EMAIL"'"); \
    page.loginWidget.PasswordLineEdit.setText("'"$QT_PASSWORD"'"); \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.IntroductionPageCallback = function() { \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.TargetDirectoryPageCallback = function() { \
    gui.currentPageWidget().TargetDirectoryLineEdit.setText("/opt/Qt/install"); \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.ComponentSelectionPageCallback = function() { \
    function list_packages() { \
      var components = installer.components(); \
      console.log("Available components: " + components.length); \
      var packages = ["Packages: "]; \
      for (var i = 0 ; i < components.length ;i++) { \
          packages.push(components[i].name); \
      } \
      console.log(packages.join(" ")); \
    } \
\
    list_packages(); \
\
    var widget = gui.currentPageWidget(); \
    widget.deselectAll(); \
    widget.selectComponent("qt.qt5.5123.gcc_64"); \
\
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.LicenseAgreementPageCallback = function() { \
    gui.currentPageWidget().AcceptLicenseRadioButton.setChecked(true); \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.StartMenuDirectoryPageCallback = function() { \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.ReadyForInstallationPageCallback = function() { \
    gui.clickButton(buttons.NextButton); \
} \
\
Controller.prototype.PerformInstallationPageCallback = function() { \
    gui.clickButton(buttons.CommitButton); \
} \
\
Controller.prototype.FinishedPageCallback = function() { \
    var checkBoxForm = gui.currentPageWidget().LaunchQtCreatorCheckBoxForm; \
    if (checkBoxForm && checkBoxForm.launchQtCreatorCheckBox) { \
        checkBoxForm.launchQtCreatorCheckBox.checked = false; \
    } \
    gui.clickButton(buttons.FinishButton); \
}' > qt-installer-nonintaractive.qs

# Qtをインストール
RUN cd /opt/Qt && \
    chmod +x qt-opensource-linux-x64-5.12.3.run && \
    chmod +x qt-installer-nonintaractive.qs && \
    ./qt-opensource-linux-x64-5.12.3.run --script qt-installer-nonintaractive.qs --verbose --platform minimal && \
    ln -s /opt/Qt/install/5.12.3 /opt/Qt/5 && \
    # インストール完了後、認証情報を含むファイルを削除
    rm -f qt-installer-nonintaractive.qs

# ステージ2: 最終イメージの構築
FROM --platform=linux/amd64 ubuntu:18.04
# 注: --platform フラグは Qt インストーラーが x86 専用のため必要

RUN apt-get clean && apt-get update && \
    apt-get install -y wget git vim curl gnupg2 lsb-release iproute2

# Prepare for Gazebo
#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
#RUN apt-get update

# Install Python3 packages
RUN apt-get install -y python3 python3-pip
RUN pip3 install rospkg 
RUN pip3 install empy==3.3.4
RUN pip3 install psutil

# Time zone settings
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
ENV TZ=Asia/Tokyo

# Install ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get -y install ros-melodic-desktop-full
RUN apt-get -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install gazebo packages
RUN apt-get install -y ros-melodic-gazebo-*

RUN sed -i '/ROS_PYTHON_VERSION/s/^/#/; /ROS_PYTHON_VERSION/a export ROS_PYTHON_VERSION=3' /opt/ros/melodic/etc/catkin/profile.d/1.ros_python_version.sh

# Install Netwide Assembler (NASM)
RUN cd /usr/local/src && \
    wget https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/nasm-2.15.05.tar.gz && \
    tar zxvf nasm-2.15.05.tar.gz && \
    cd nasm-2.15.05 && \
    ./configure && \
    make install

# Install Video Reception Environment
RUN git clone https://code.videolan.org/videolan/x264.git /usr/local/src/x264-master && \
    cd /usr/local/src/x264-master && \
    ./configure --disable-asm --enable-shared --enable-static --enable-pic && \
    make install

RUN git clone https://github.com/FFmpeg/FFmpeg.git -b n4.1.3 /usr/local/src/ffmpeg-4.1.3 && \
    cd /usr/local/src/ffmpeg-4.1.3 && \
    ./configure --extra-cflags="-I/usr/local/include" \
                --extra-ldflags="-L/usr/local/lib" \
                --extra-libs="-lpthread -lm -ldl -lpng" \
                --enable-pic \
                --disable-programs \
                --enable-shared \
                --enable-gpl \
                --enable-libx264 \
                --enable-encoder=png \
                --enable-version3 && \
    make install -j$(nproc)

# Install VLC Media Player
RUN apt-get install -y libasound2-dev libxcb-shm0-dev libxcb-xv0-dev \
                libxcb-keysyms1-dev libxcb-randr0-dev libxcb-composite0-dev \
                lua5.2 lua5.2-dev protobuf-compiler bison libdvbpsi-dev libpulse-dev
RUN cd /usr/local/src && \
    wget https://get.videolan.org/vlc/3.0.7.1/vlc-3.0.7.1.tar.xz && \
    tar Jxvf vlc-3.0.7.1.tar.xz && \
    cd /usr/local/src/vlc-3.0.7.1 && \
    cp ./share/vlc.appdata.xml.in.in ./share/vlc.appdata.xml && \
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
                --enable-dvbpsi && \
    make install -j$(nproc)&& \
    ln -s /usr/local/src/vlc-3.0.7.1 /usr/local/src/vlc

# Qtインストール済みディレクトリをコピー（認証情報を含まない）
COPY --from=qt-builder /opt/Qt /opt/Qt

# Install Font File
RUN apt-get install -y fonts-roboto

# Install Docker
RUN apt-get update && \
    apt-get install -y init systemd
RUN apt-get install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN apt-get update
RUN apt-get install -y docker-ce docker-ce-cli containerd.io
RUN pip3 install docker defusedxml netifaces

# Download and build int-ball2_simulator
#RUN mkdir -p /home/nvidia
#WORKDIR /home/nvidia
WORKDIR /home
#RUN git clone https://github.com/jaxa/int-ball2_simulator.git IB2
RUN git clone https://github.com/jaxa/int-ball2_simulator.git

#RUN sed -i 's/^intball2_telecommand_target_ip:.*$/intball2_telecommand_target_ip: [127.0.0.1]/' /home/nvidia/IB2/Int-Ball2_platform_gse/src/ground_system/communication_software/config/params.yml
#RUN sed -i 's/^intball2_telecommand_target_port:.*$/intball2_telecommand_target_port: [23456]/' /home/nvidia/IB2/Int-Ball2_platform_gse/src/ground_system/communication_software/config/params.yml
#RUN sed -i 's/^intball2_telemetry_receive_port:.*$/intball2_telemetry_receive_port: 34567/' /home/nvidia/IB2/Int-Ball2_platform_gse/src/ground_system/communication_software/config/params.yml



# Download int-ball2_platform_works repository
#WORKDIR /home/nvidia
COPY platform_works /home/nvidia/platform_works

#RUN sed -i 's/<arg name="receive_port" default="[^"]*"/<arg name="receive_port" default="23456"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/bringup.launch
#RUN sed -i 's/<arg name="ocs_host" default="[^"]*"/<arg name="ocs_host" default="localhost"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/bringup.launch
#RUN sed -i 's/<arg name="ocs_port" default="[^"]*"/<arg name="ocs_port" default="34567"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/flight_software/trans_communication/launch/bringup.launch

#RUN sed -i 's/<arg name="container_ros_master_uri" default="[^"]*"/<arg name="container_ros_master_uri" default="http:\/\/localhost:11311"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch
#RUN sed -i 's/<arg name="host_ib2_workspace" default="[^"]*"/<arg name="host_ib2_workspace" default="\/home\/nvidia\/IB2"/' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch

RUN cd /home/nvidia/IB2/Int-Ball2_platform_gse && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make"
#RUN mkdir /var/log/ground_system && chown $USER:$USER /var/log/ground_system
RUN apt-get install -y libpcl-dev ros-melodic-pcl-ros && \
    cd /home/nvidia/IB2/Int-Ball2_platform_simulator && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make -DWITH_PCA9685=OFF"


# 初期化スクリプトの追加
COPY init-container.sh /
RUN chmod +x /init-container.sh

# コンテナ起動時のデフォルトコマンド
CMD ["/init-container.sh"]

