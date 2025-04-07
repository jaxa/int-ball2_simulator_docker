## クイックスタート

### ビルド方法

```bash
# Windows PowerShell または コマンドプロンプト
docker build --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator .

# Linux/macOS
docker build --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator .
```


git clone --recursive https://github.com/AkihikoHONDA/ib2sim_docker_test.git

vcxsrv
「Native OpenGL」オプションにチェックを入れる
「Additional parameters for VcXsrv」の欄に -wgl を追加



docker-compose up -d

docker exec -it ib2_simulator bash

cd /home/int-ball2_simulator/Int-Ball2_platform_gse
source devel/setup.bash
roslaunch platform_gui bringup.launch

cd int-ball2_simulator/Int-Ball2_platform_simulator

source devel/setup.bash
rosrun platform_sim_tools simulator_bringup.sh


docker-compose down


simとplatformの位置をきれいにしないと

### 実行方法

```bash
# 方法1: Docker Composeを使用（推奨）
docker-compose up -d

# 方法2: Docker コマンドを直接使用
docker run -it --privileged \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY=$DISPLAY \
  ib2_simulator
```

### コンテナに接続

```bash
docker exec -it ib2_simulator bash
```

