## クイックスタート

### ビルド方法

```bash
# Windows PowerShell または コマンドプロンプト
docker build --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator .

# Linux/macOS
docker build --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator .
```

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
