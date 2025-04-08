#!/bin/bash

# エラー発生時に標準エラー出力に情報を表示
set -e

# 色の設定
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Int-Ball2シミュレータコンテナを初期化しています...${NC}"

# システムサービスの開始
if [ -x "$(command -v systemctl)" ]; then
  echo "systemdを使用してサービスを開始しています..."
  systemctl start docker
else
  echo "service コマンドを使用してサービスを開始しています..."
  service docker start
fi

# Dockerデーモンの起動を待機（短縮版）
echo "Dockerデーモンの起動を待機しています..."
COUNTER=0
MAX_TRIES=10
until docker info >/dev/null 2>&1; do
  COUNTER=$((COUNTER+1))
  if [ $COUNTER -gt $MAX_TRIES ]; then
    echo -e "${YELLOW}警告: Dockerデーモンの起動タイムアウト${NC}"
    echo "続行しますが、Docker機能は利用できない可能性があります"
    break
  fi
  echo "Dockerデーモンの起動を待機中... ($COUNTER/$MAX_TRIES)"
  sleep 1
done

if docker info >/dev/null 2>&1; then
  echo -e "${GREEN}Dockerデーモンが起動しました${NC}"
  
  # ib2_userイメージの存在確認
  if docker image inspect ib2_user:0.1 &>/dev/null; then
    echo -e "${GREEN}ib2_userイメージが見つかりました${NC}"
  else
    echo -e "${YELLOW}警告: ib2_userイメージが見つかりません。事前にビルドが必要です。${NC}"
    echo "ホスト側で以下のコマンドを実行してください："
    echo "cd /path/to/int-ball2_platform_works/platform_docker/template && docker build -t ib2_user:0.1 ."
  fi
else
  echo -e "${YELLOW}警告: Dockerデーモンが起動していないため、Docker機能は利用できません${NC}"
fi

# ROS環境変数の設定
source /opt/ros/melodic/setup.bash
if [ -f "/home/int-ball2_simulator/Int-Ball2_platform_gse/devel/setup.bash" ]; then
  source /home/int-ball2_simulator/Int-Ball2_platform_gse/devel/setup.bash
fi
if [ -f "/home/int-ball2_simulator/Int-Ball2_platform_simulator/devel/setup.bash" ]; then
  source /home/int-ball2_simulator/Int-Ball2_platform_simulator/devel/setup.bash
fi

# 環境情報の表示
echo -e "${GREEN}コンテナの初期化が完了しました${NC}"
echo "=========================================="
echo "Int-Ball2 シミュレータ情報:"
echo " - ROSバージョン: $(rosversion -d)"
echo " - Qtバージョン: 5.12.3"
echo " - Dockerバージョン: $(docker --version)"
echo "=========================================="
echo -e "${YELLOW}注意: GUI操作には -e DISPLAY 設定と X11ソケットのマウントが必要です${NC}"
echo ""

# 引数がある場合は、その引数を使ってコマンドを実行
# そうでなければ、対話型シェルを起動
if [ $# -gt 0 ]; then
  echo "指定されたコマンドを実行します: $@"
  exec "$@"
else
  # .bashrcにROSの設定を追加
  grep -q "source /opt/ros/melodic/setup.bash" ~/.bashrc || echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  
  # 対話型シェルを起動
  exec /bin/bash
fi
