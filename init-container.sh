#!/bin/bash

# コンテナ初期化スクリプト

# システムサービスの開始
if [ -x "$(command -v systemctl)" ]; then
  echo "systemdを使用してサービスを開始しています..."
  systemctl start docker
else
  echo "service コマンドを使用してサービスを開始しています..."
  service docker start
fi

# Dockerデーモンの起動を待機
echo "Dockerデーモンの起動を待機しています..."
COUNTER=0
MAX_TRIES=30
until docker info >/dev/null 2>&1; do
  COUNTER=$((COUNTER+1))
  if [ $COUNTER -gt $MAX_TRIES ]; then
    echo "Dockerデーモンの起動タイムアウト"
    exit 1
  fi
  echo "Dockerデーモンの起動を待機中... ($COUNTER/$MAX_TRIES)"
  sleep 1
done

echo "Dockerデーモンが起動しました"

# Int-Ball2 ユーザイメージのビルド
if [ -d "/home/int-ball2_platform_works/platform_docker/template" ]; then
  echo "Int-Ball2 ユーザイメージのビルドを開始します..."
  cd /home/int-ball2_platform_works/platform_docker/template
  docker build . -t ib2_user:0.1
  
  if [ $? -eq 0 ]; then
    echo "Int-Ball2 ユーザイメージのビルドが完了しました"
  else
    echo "警告: Int-Ball2 ユーザイメージのビルドに失敗しました"
  fi
else
  echo "警告: Int-Ball2 プラットフォームワークディレクトリが見つかりません"
fi

# ユーザーが指定したコマンドを実行
if [ $# -gt 0 ]; then
  echo "指定されたコマンドを実行します: $@"
  exec "$@"
else
  echo "対話型シェルを起動します"
  exec /bin/bash
fi