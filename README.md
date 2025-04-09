# Int-Ball2 Simulator Docker環境

このリポジトリ（int-ball2_sim_docker）は、JAXA Int-Ball2シミュレータとその中のユーザー制御プログラムをDockerイメージ化する手順を提供します。

## 概要

Int-Ball2シミュレータとユーザープログラムを個別のDockerコンテナとして実行し、Docker Compose を用いて連携動作させます。ROS（Robot Operating System）を使用したマイクロ重力環境下でのユーザープログラムによるロボット動作をシミュレーションできます。

**現在の検証環境**: Windows + Docker Desktop

## Docker-in-Docker (DooD) アーキテクチャ

このプロジェクトでは「Docker-outside-of-Docker (DooD)」アーキテクチャを採用しています。この方式では：

1. ホストマシン上のDocker Engineを、シミュレータコンテナ内からも利用できるようにしています
2. シミュレータコンテナがユーザープログラムコンテナを制御・管理します
3. `/var/run/docker.sock`をマウントすることで、シミュレータコンテナからホストのDocker APIにアクセスし、ユーザープログラムコンテナを起動・停止できます

この仕組みにより、シミュレータとユーザープログラムを分離しつつも、連携して動作させることが可能になっています。

## セットアップ手順

### 1. リポジトリのクローンとサブモジュールの初期化

```bash
# リポジトリのクローン(サブモジュールも含む)
git clone --recursive https://github.com/jaxa/int-ball2_simulator_docker.git
cd int-ball2_simulator_docker
```

既にクローン済みのサブモジュールを最新版に更新する場合:

```bash
git submodule update --remote
```



### 2. ユーザープログラムの用意
ユーザープログラムは、`int-ball2_simulator_docker/IB2/Int-Ball2_platform_simulator/src/user/`以下に配置してください。

このコードはシミュレータビルド時に使用します。

### 3. シミュレータのDockerイメージビルド
Qtをインストールするために、Qtのアカウント情報を用意してください。

Qtのアカウント名（メールアドレス）・パスワードを、QT_EMAIL・QT_PASSWORDの引数として渡してください。

```bash
# シミュレータイメージのビルド（初回は30分以上かかる場合があります）
docker build --build-arg QT_EMAIL=your.email@example.com --build-arg QT_PASSWORD=your_password -t ib2_simulator:latest .
```

### 4. platfor_worksのDockerイメージビルド
platform_worksのDockerイメージをビルドします。

任意の位置で実施して下さい。

```bash
# ユーザープログラムイメージのビルド
git clone https://github.com/jaxa/int-ball2_platform_works.git platform_works
cd platform_works/platform_docker/template
docker build -t ib2_user:0.1 .
```

### 4. Docker Composeによる実行

```bash
# コンテナの起動
cd int-ball2_simulator_docker
docker-compose up -d

# シミュレータコンテナのbashに接続
docker exec -it ib2_simulator bash

# コンテナの停止と削除
docker-compose down
```

## シミュレータコンテナ内の基本操作

シミュレータコンテナに接続した後、ROSコマンドを実行できます:

```bash
docker exec -it ib2_simulator bash

# Docker内で実行
source /opt/ros/melodic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_gse/devel/setup.bash
roslaunch platform_gui bringup.launch
```

別のターミナルで
```bash
docker exec -it ib2_simulator bash

# Docker内で実行
source /opt/ros/melodic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash
rosrun platform_sim_tools simulator_bringup.sh
```

## Windows環境での注意事項

- Docker Desktopの設定で、「WSL Integration」が有効になっていることを確認してください。
- GUIを表示するには、X11サーバー（VcXsrvなど）が必要です。
- VcXsrvでは以下の設定をしてください。
  - XLaunchを起動して次の設定をすること：Multiple windowsを選択、Start no clientを選択、Clipboard/Native opengl/Disable access controlにチェックを入れる、Additional parameters for VcXsrvの欄に、-wglを記入

## Linux環境での注意事項（TODO）

- Linuxで実行する場合、X11表示のために以下のコマンドを実行する必要があります:
  ```bash
  xhost +local:docker
  ```
- docker-compose.ymlの`DISPLAY`環境変数を`DISPLAY=:0`に変更してください。
- NVIDIAドライバーがインストールされている場合、nvidia-container-runtimeの設定も必要になることがあります。

## トラブルシューティング

### シミュレータが起動しない場合

- Docker logを確認して詳細なエラーメッセージを確認してください:
  ```bash
  docker logs ib2_simulator
  ```

## TODO リスト

- [ ] Linuxでの動作検証
- [ ] サンプルユーザープログラムの追加


## その他
* Arm CPUでの動作も考慮して、ベースイメージを--platform=linux/amd64で取得しています。これに関して警告が出ますが問題ありません。

## ライセンス情報

Int-Ball2シミュレータのライセンスについては[JAXA公式リポジトリ](https://github.com/jaxa/int-ball2_simulator)を参照してください。

## 貢献方法

バグ報告や機能改善の提案は、Issuesセクションにお寄せください。プルリクエストも歓迎します。

---

**注意**: このプロジェクトは開発中のため、予告なく変更される場合があります。




