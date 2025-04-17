# Int‑Ball2 シミュレータ Docker 環境 🚀

本リポジトリ **int‑ball2_simulator_docker** は、JAXA Int‑Ball2 シミュレータとユーザープログラム (制御ノード) を Docker イメージ化し、Docker Compose で連携動作させるための環境を提供します。  
マイクロ重力環境下でのロボット挙動を手軽にシミュレーションできます。💫

---

## 概要 / Overview ✨

- **目的**: Int‑Ball2 シミュレータ + ユーザープログラムをワンコマンドで起動  
- **検証環境**: Windows 11 + WSL2 (Ubuntu 24.04)  
- **主要技術**: Docker‑outside‑of‑Docker (DooD)、ROS、X11 / WSLg

---

## アーキテクチャ概要 🖼️

```mermaid
C4Context
title Docker Int‑Ball2 Simulator Architecture

Enterprise_Boundary(host, "Host Environment") {
    System(dockerService, "Docker Service", "Manages and runs containers")
    System(userProgram, "User Program", "Program developed by users")
    
    Container_Boundary(simulatorContainer, "Simulator Container") {
        Component(gse, "GSE", "Ground Support Equipment")
        Component(rvizGazebo, "RViz+Gazebo", "Simulation environment")
    }
    
    Container_Boundary(userProgramContainer, "User Program Container") {
        Component(cmdsh, "cmd.sh", "Execution script")
    }
}

Rel(gse, userProgramContainer, "Run", "Executes program")
Rel(gse, rvizGazebo, "CMD", "Sends commands")
Rel(rvizGazebo, gse, "TLM", "Sends telemetry")
Rel(cmdsh, userProgram, "Run", "Executes program")
```

1. **シミュレータコンテナ** からホストの Docker Engine を操作  
2. `/var/run/docker.sock` を共有し **ユーザープログラムコンテナ** を生成・管理  
3. GUI 表示は X11 / WSLg 経由でホストの画面へ出力  

---

## 前提条件 / Prerequisites 📝

- **Docker** と **Docker Compose** がインストール済み  
- **Qt アカウント**（メールアドレス & パスワード）  
- シミュレータ GUI 表示のための **X11/WSLg** 環境

> **備考**: Qt ライセンス登録は無料です。  

---

## セットアップと実行手順 💻

### 1. リポジトリのクローン

```bash
git clone https://github.com/jaxa/int-ball2_simulator_docker.git
cd int-ball2_simulator_docker
```

### 2. 共有ディレクトリの作成

```bash
mkdir -p shared_data_sim
```

### 3. ユーザープログラムの配置

ユーザープログラムの ROS パッケージを  
`ib2_user_ws/src/user/` に配置します。サンプルを用意しても構いません。

### 4. シミュレータ Docker イメージのビルド

```bash
docker build --build-arg HOST_USER_PATH="$(pwd)" --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator:latest .
```

> **注意**: 初回ビルドは 60 分以上かかる場合があります。

### 5. ユーザープログラムのビルド

```bash
docker run --rm \
  -v "$(pwd)/ib2_user_ws:$(pwd)/ib2_user_ws" \
  ib2_simulator:latest \
  bash -c "source /opt/ros/melodic/setup.bash && \
           source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash && \
           cd $(pwd)/ib2_user_ws && catkin_make"
```

### 6. platform_works イメージのビルド

```bash
git clone https://github.com/jaxa/int-ball2_platform_works.git platform_works
cd platform_works/platform_docker/template
docker build -t ib2_user:0.1 .
cd -
```

### 7. Docker Compose で起動

```bash
# シミュレータ & ユーザープログラムをバックグラウンド起動
docker compose up -d

# シミュレータコンテナに入る場合
docker exec -it ib2_simulator bash
```

---

## シミュレータの実行手順 🕹️

### ターミナル 1: GUI 起動

```bash
source /opt/ros/melodic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_gse/devel/setup.bash
roslaunch platform_gui bringup.launch
```

### ターミナル 2: シミュレータ起動

```bash
docker exec -it ib2_simulator bash

# コンテナ内
source /opt/ros/melodic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash
rosrun platform_sim_tools simulator_bringup.sh
```

---

## ユーザープログラムの更新方法 🔄

プログラム変更後は再ビルドが必要です。

```bash
docker run --rm \
  -v "$(pwd)/ib2_user_ws:$(pwd)/ib2_user_ws" \
  ib2_simulator:latest \
  bash -c "source /opt/ros/melodic/setup.bash && \
           source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash && \
           cd $(pwd)/ib2_user_ws && catkin_make"
docker compose restart        # 必要に応じて
```

---

## プラットフォーム別の設定 ⚙️

### Windows + WSL2

- GUI 表示は **WSLg** を利用  
- `docker-compose.yml` の環境変数は既定で以下を設定  
  ```yaml
  environment:
    - DISPLAY=:0
    - LIBGL_ALWAYS_INDIRECT=0
    - QT_X11_NO_MITSHM=1
    - MESA_GL_VERSION_OVERRIDE=3.3
  ```
- NVIDIA GPU を利用したい場合は `runtime: nvidia` のコメントを外してください。

### Linux

- X11 許可:
  ```bash
  xhost +local:docker
  ```
- `DISPLAY=:0` (または Wayland 環境に応じた値) を設定してください。

---

## トラブルシューティング 🛠️

| 症状 | よくある原因 | 解決策 |
| ---- | ------------ | ------ |
| `Error: No such container: ib2_simulator` | コンテナ未起動 | `docker compose up -d` を実行 |
| `Qt: cannot connect to X server` | DISPLAY 設定不一致 | ホスト / コンテナ双方の `$DISPLAY` を確認 |
| ROS セットアップエラー | 環境スクリプト未読込 | `source /opt/ros/melodic/setup.bash` を実行 |
| 画面が表示されない | X11 ソケットマウント漏れ | `/tmp/.X11-unix:` マウントを確認 |

詳細ログは:

```bash
# コンテナログ
docker logs ib2_simulator

# X11 変数確認
echo $DISPLAY                      # ホスト
docker exec ib2_simulator bash -c 'echo $DISPLAY'
```

---

## 高度な使用方法 🌐

### カスタマイズ可能なパラメータ

- **HOST_USER_PATH**: ホストワークスペースを指すパス  
- **ボリュームマウント**: 共有ディレクトリや X11 ソケット  
- **環境変数**: DISPLAY / GPU 切替など

### 開発のヒント

- ROS ワークスペースは `catkin_make` 後に **devel/setup.bash** を source  
- Int‑Ball2 API 仕様を確認し、挙動・座標系を把握してから制御ロジックを実装

---

## ライセンス情報 📜

Int‑Ball2 シミュレータのライセンスは [JAXA 公式リポジトリ](https://github.com/jaxa/int-ball2_simulator) を参照してください。

---

## 貢献方法 🤝

不具合報告や機能提案は **Issues** へ、コード修正は **Pull Request** を歓迎します。


> **注意**: 本プロジェクトは開発中であり、仕様は予告なく変更される場合があります。


