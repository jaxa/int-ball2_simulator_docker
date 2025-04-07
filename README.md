## 使用方法

### ステップ1: Dockerイメージのビルド

```bash
# 実行権限を付与
chmod +x generate-qt-installer.sh build.sh init-container.sh run-container.sh

# イメージのビルド
./build.sh your.email@example.com your_password
```

**注意**: パスワードはコマンドライン履歴に残る可能性があります。`history -c`でコマンド履歴を消去できます。

### ステップ2: シミュレータの実行

#### オプション1: run-container.sh スクリプトを使用（推奨）

```bash
# 対話モードで実行
./run-container.sh

# または、バックグラウンドで実行
./run-container.sh --detach
```

#### オプション2: Docker Composeを使用

```bash
docker compose up
```

#### オプション3: Docker コマンドを直接使用

```bash
docker run -it --privileged \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY=$DISPLAY \
  ib2_simulator /init-container.sh
```

## ファイル構成

- `Dockerfile`: マルチステージビルドによるイメージ定義
- `generate-qt-installer.sh`: Qtインストーラー用スクリプト生成
- `build.sh`: セキュアなイメージビルドプロセス
- `init-container.sh`: コンテナ起動時の初期化スクリプト
- `run-container.sh`: コンテナ実行用スクリプト
- `docker-compose.yml`: Docker Compose設定
- `.gitignore`: 機密ファイルのリポジトリへの登録を防止



