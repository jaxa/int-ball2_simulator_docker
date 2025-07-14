# Int‑Ball2 シミュレータ Docker 環境 🚀

<p style="display: inline">
  <img src="https://img.shields.io/badge/-Docker-1488C6.svg?logo=docker&style=flat">
  <img src="https://img.shields.io/badge/ROS-darkblue?logo=ros">
</p>


 **int‑ball2_simulator_docker** は、[JAXA Int‑Ball2 シミュレータ](https://github.com/jaxa/int-ball2_simulator)とユーザープログラム (制御ノード) を Docker イメージ化し、Docker Compose で連携動作させるための環境を提供します。  
マイクロ重力環境下での飛行ロボットの挙動を手軽にシミュレーションできます。💫

---

## 概要 / Overview ✨

- **目的**: Int‑Ball2 シミュレータ + ユーザープログラムを簡単なコマンドで起動  
- **検証環境**: Windows 11 + WSL2 (Ubuntu 24.04)  
- **主要技術**: Docker‑outside‑of‑Docker (DooD)、ROS、X11 / WSLg

---

## アーキテクチャ概要 🖼️

  ```mermaid
  graph TB
      subgraph HostEnvironment
          dockerService["Docker Service"]
          userProgram["User Program"]
        
          subgraph SimulatorContainer
              gse["GSE"]
              rvizGazebo["RViz+Gazebo"]
          end
        
          subgraph UserProgramContainer
              cmdsh["cmd.sh"]
          end
      end
    
      dockerService -- "Run" --> SimulatorContainer
      gse -- "Docker Run" --> UserProgramContainer
      gse <-- "CMD/TLM" --> rvizGazebo
      cmdsh -- "Run" --> userProgram
      userProgram -- "Control" --> rvizGazebo
  ```

1. ホスト環境に、**シミュレータコンテナ** 、**ユーザープログラムコンテナ** 、**ユーザープログラム** を配置
2. **シミュレータコンテナ** の GSE からホストの Docker Engine を操作し、**ユーザープログラムコンテナ**を起動  
3. `/var/run/docker.sock` を共有し **ユーザープログラムコンテナ** を生成・管理
4. **ユーザープログラムコンテナ** はホスト環境にあるユーザープログラムを起動し**シミュレータコンテナ** のInt-Ball2モデルを操作
3. GUI 表示は X11 / WSLg 経由でホストの画面へ出力  

---

## 前提条件 / Prerequisites 📝

- ホスト環境に**Docker** と **Docker Compose** がインストール済み  
- [**Qt アカウント**](https://login.qt.io/login)（メールアドレス & パスワード）  
- シミュレータ GUI 表示のための **X11/WSLg** 環境

> **備考**: Qt ライセンス登録は無料です。  

---

## セットアップと実行手順 💻

### 1. このリポジトリのクローン

```bash
git clone https://github.com/jaxa/int-ball2_simulator_docker.git
cd int-ball2_simulator_docker
```

### 2. ファイル共有ディレクトリの作成

```bash
mkdir -p shared_data_sim
```

### 3. ユーザープログラムの配置

ユーザープログラムの ROS パッケージを`int-ball2_simulator_docker/ib2_user_ws/src/user/` に配置します。

### 4. シミュレータ Docker イメージのビルド
**your.email@example.com** と **your_password** をQtアカウント情報で置き換えてください。

```bash
docker build --build-arg HOST_USER_PATH="$(pwd)" --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator:latest .
```

> **注意**: 初回ビルドは 60 分以上かかる場合があります。

### 5. ユーザープログラムのビルド
シミュレータコンテナのROSシステムを使用して、ホスト環境にあるユーザープログラムをビルドします。

```bash
docker run --rm \
  -v "$(pwd)/ib2_user_ws:$(pwd)/ib2_user_ws" \
  ib2_simulator:latest \
  bash -c "source /opt/ros/melodic/setup.bash && \
           source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash && \
           cd $(pwd)/ib2_user_ws && catkin_make"
```

### 6. platform_works イメージのビルド
ユーザープログラムイメージをビルドします。

```bash
git clone https://github.com/jaxa/int-ball2_platform_works.git platform_works
cd platform_works/platform_docker/template
docker build -t ib2_user:0.1 .
cd -
```

### 7. Docker Compose シミュレータコンテナを起動

```bash
# シミュレータ & ユーザープログラムをバックグラウンド起動
cd int-ball2_simulator_docker
docker compose up -d

# シミュレータコンテナに入る場合
docker exec -it ib2_simulator bash
```

---

## シミュレータの実行手順 🕹️

### ターミナル 1: GSE 起動

```bash
# コンテナ内
source /opt/ros/melodic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_gse/devel/setup.bash
roslaunch platform_gui bringup.launch
```

### ターミナル 2: シミュレータ起動
別のターミナルで以下を実行

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
    - DISPLAY
    # - NVIDIA_VISIBLE_DEVICES=all
    # - NVIDIA_DRIVER_CAPABILITIES=all
    - LIBGL_ALWAYS_INDIRECT=0
    - QT_X11_NO_MITSHM=1
    - MESA_GL_VERSION_OVERRIDE=3.3
  ```
- NVIDIA GPU を利用したい場合は `runtime: nvidia`、`NVIDIA_VISIBLE_DEVICES=all`、`NVIDIA_DRIVER_CAPABILITIES=all` のコメントを外してください。

### Linux

- ib2_simulatorコンテナを立ち上げるターミナルで以下を入力し、X11へのアクセスを許可しておく。
  ```bash
  xhost +local:docker
  ```

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


