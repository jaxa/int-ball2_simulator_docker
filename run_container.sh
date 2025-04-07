#!/bin/bash

# エラー発生時に処理を停止
set -e

# 色の設定
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ヘルプメッセージの表示
function show_help {
  echo "Int-Ball2シミュレータコンテナの実行スクリプト"
  echo ""
  echo "使用方法: $0 [オプション]"
  echo ""
  echo "オプション:"
  echo "  -h, --help         このヘルプメッセージを表示"
  echo "  -n, --name NAME    コンテナ名を指定（デフォルト: ib2_simulator）"
  echo "  -d, --detach       デタッチモードでコンテナを実行"
  echo "  -i, --image IMAGE  使用するイメージを指定（デフォルト: ib2_simulator:latest）"
  echo ""
  echo "例:"
  echo "  $0"
  echo "  $0 --detach"
  echo "  $0 --name my_simulator --image ib2_simulator:dev"
}

# デフォルト値
CONTAINER_NAME="ib2_simulator"
DETACH=false
IMAGE_NAME="ib2_simulator:latest"
INIT_SCRIPT="/init-container.sh"

# 引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    -n|--name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    -d|--detach)
      DETACH=true
      shift
      ;;
    -i|--image)
      IMAGE_NAME="$2"
      shift 2
      ;;
    *)
      echo -e "不明な引数: $1"
      show_help
      exit 1
      ;;
  esac
done

# イメージの存在確認
if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo -e "${YELLOW}警告: イメージ ${IMAGE_NAME} が見つかりません${NC}"
  read -p "ビルドスクリプトを実行しますか？ (y/N): " RUN_BUILD
  if [[ "$RUN_BUILD" =~ ^[Yy]$ ]]; then
    echo "ビルドスクリプトを実行するには、Qtアカウント情報が必要です"
    read -p "Qtアカウントのメールアドレス: " QT_EMAIL
    read -sp "Qtアカウントのパスワード: " QT_PASSWORD
    echo ""
    
    ./build.sh "$QT_EMAIL" "$QT_PASSWORD"
    
    if [ $? -ne 0 ]; then
      echo "ビルドに失敗しました。スクリプトを終了します。"
      exit 1
    fi
  else
    echo "イメージが見つからないため、スクリプトを終了します。"
    exit 1
  fi
fi

# init-container.shスクリプトをコンテナにコピー
echo -e "${GREEN}初期化スクリプトをコンテナにコピーします...${NC}"
TEMP_CONTAINER=$(docker create "$IMAGE_NAME")
docker cp init-container.sh "$TEMP_CONTAINER":/init-container.sh
docker commit "$TEMP_CONTAINER" "$IMAGE_NAME"
docker rm "$TEMP_CONTAINER" > /dev/null

# 実行中のコンテナの確認
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo -e "${YELLOW}警告: コンテナ ${CONTAINER_NAME} はすでに存在します${NC}"
  read -p "既存のコンテナを削除しますか？ (y/N): " REMOVE_CONTAINER
  if [[ "$REMOVE_CONTAINER" =~ ^[Yy]$ ]]; then
    echo "既存のコンテナを削除しています..."
    docker rm -f "$CONTAINER_NAME" > /dev/null
  else
    echo "既存のコンテナを使用します。スクリプトを終了します。"
    exit 0
  fi
fi

# コンテナの実行
echo -e "${GREEN}Int-Ball2シミュレータコンテナを起動しています...${NC}"

# デタッチモードの設定
if [ "$DETACH" = true ]; then
  DOCKER_MODE="-d"
  echo "コンテナはバックグラウンドで実行されます"
else
  DOCKER_MODE="-it"
  echo "コンテナは対話モードで実行されます"
fi

# コンテナを実行
docker run $DOCKER_MODE \
  --name "$CONTAINER_NAME" \
  --privileged \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/.Xauthority:/root/.Xauthority" \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  --network host \
  --ipc host \
  "$IMAGE_NAME" /init-container.sh

echo -e "${GREEN}コンテナが正常に起動しました${NC}"
if [ "$DETACH" = true ]; then
  echo "コンテナに接続するには: docker exec -it $CONTAINER_NAME bash"
fi