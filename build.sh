#!/bin/bash

# エラー発生時に処理を停止
set -e

# 色の設定
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ヘルプメッセージの表示
function show_help {
  echo "Int-Ball2シミュレータのセキュアなビルドスクリプト"
  echo ""
  echo "使用方法: $0 [オプション] <メールアドレス> <パスワード>"
  echo ""
  echo "オプション:"
  echo "  -h, --help         このヘルプメッセージを表示"
  echo "  -n, --no-cleanup   ビルド後のクリーンアップをスキップ"
  echo "  -f, --force        既存のqsファイルがある場合も上書き"
  echo "  -v, --verbose      詳細な出力を表示"
  echo ""
  echo "例:"
  echo "  $0 your.email@example.com yourpassword"
  echo ""
  echo "セキュリティ情報:"
  echo "  * このスクリプトは認証情報を含むファイルを一時的に作成します"
  echo "  * ビルド後、一時ファイルは削除され、Dockerイメージからも認証情報は削除されます"
  echo "  * ただし、コマンド履歴に認証情報が残る可能性があるため注意してください"
}

# デフォルト値
CLEANUP=true
FORCE=false
VERBOSE=false

# 引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    -n|--no-cleanup)
      CLEANUP=false
      shift
      ;;
    -f|--force)
      FORCE=true
      shift
      ;;
    -v|--verbose)
      VERBOSE=true
      shift
      ;;
    *)
      # 位置引数として処理
      if [ -z "$QT_EMAIL" ]; then
        QT_EMAIL=$1
      elif [ -z "$QT_PASSWORD" ]; then
        QT_PASSWORD=$1
      else
        echo -e "${RED}エラー: 不明な引数 '$1'${NC}"
        show_help
        exit 1
      fi
      shift
      ;;
  esac
done

# 必須パラメータのチェック
if [ -z "$QT_EMAIL" ] || [ -z "$QT_PASSWORD" ]; then
  echo -e "${RED}エラー: Qtアカウントのメールアドレスとパスワードが必要です${NC}"
  show_help
  exit 1
fi

# 詳細モードの設定
if [ "$VERBOSE" = true ]; then
  set -x
  DOCKER_VERBOSE="--progress=plain"
else
  DOCKER_VERBOSE=""
fi

# すでにqsファイルがある場合のチェック
if [ -f "qt-installer-nonintaractive.qs" ] && [ "$FORCE" = false ]; then
  echo -e "${YELLOW}警告: qt-installer-nonintaractive.qs がすでに存在します${NC}"
  read -p "上書きしますか? (y/N): " OVERWRITE
  if [[ ! "$OVERWRITE" =~ ^[Yy]$ ]]; then
    echo "ビルドを中止します"
    exit 1
  fi
fi

# 一時ファイル用ディレクトリ
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

echo -e "${GREEN}ステップ 1: Qt Installerスクリプトの生成${NC}"
./generate-qt-installer.sh "$QT_EMAIL" "$QT_PASSWORD"

if [ $? -ne 0 ]; then
  echo -e "${RED}エラー: Qt Installerスクリプトの生成に失敗しました${NC}"
  exit 1
fi

# シンボリックリンクを使って認証情報の流出を防止
echo -e "${GREEN}ステップ 2: ファイルの保護設定${NC}"
chmod 600 qt-installer-nonintaractive.qs

# 一時ID（タイムスタンプベース）
TEMP_ID=$(date +%s)
TEMP_TAG="ib2_simulator_temp:$TEMP_ID"
FINAL_TAG="ib2_simulator:latest"

echo -e "${GREEN}ステップ 3: Dockerイメージのビルド（一時イメージ）${NC}"
echo "ビルド中です。これには時間がかかる場合があります..."
DOCKER_BUILDKIT=1 docker build $DOCKER_VERBOSE --platform=linux/amd64 -t "$TEMP_TAG" .

if [ $? -ne 0 ]; then
  echo -e "${RED}エラー: Dockerイメージのビルドに失敗しました${NC}"
  exit 1
fi

echo -e "${GREEN}ステップ 4: 一時ファイルのクリーンアップ${NC}"
rm -f qt-installer-nonintaractive.qs

if [ "$CLEANUP" = true ]; then
  echo -e "${GREEN}ステップ 5: セキュアなイメージの作成（履歴を除去）${NC}"
  
  # Docker exportとimportを使用して履歴を削除
  echo "一時イメージをエクスポートしています..."
  docker export $(docker create "$TEMP_TAG") > "$TEMP_DIR/ib2_simulator.tar"
  
  echo "クリーンなイメージをインポートしています..."
  cat "$TEMP_DIR/ib2_simulator.tar" | docker import - "$FINAL_TAG"
  
  echo "一時イメージと一時ファイルを削除しています..."
  docker rmi "$TEMP_TAG"
  rm -f "$TEMP_DIR/ib2_simulator.tar"
  
  echo -e "${YELLOW}注意: export/importプロセスによりエントリポイント設定が失われました${NC}"
  echo -e "${YELLOW}コンテナを実行する際は明示的にコマンドを指定してください${NC}"
else
  echo -e "${GREEN}ステップ 5: 最終イメージのタグ付け${NC}"
  docker tag "$TEMP_TAG" "$FINAL_TAG"
  
  if [ "$CLEANUP" = true ]; then
    docker rmi "$TEMP_TAG"
  else
    echo -e "${YELLOW}注意: 一時イメージ $TEMP_TAG が残っています${NC}"
  fi
fi

echo -e "${GREEN}ビルドが完了しました！${NC}"
echo "イメージ名: $FINAL_TAG"
echo ""
echo "コンテナを実行するには:"
echo "  docker run -it --privileged -v /var/run/docker.sock:/var/run/docker.sock ib2_simulator /bin/bash"
echo ""
echo -e "${YELLOW}セキュリティ注意事項:${NC}"
echo "* コマンド履歴にパスワードが残っている可能性があります (history -c で消去できます)"
echo "* 共有環境では認証情報が漏洩しないよう注意してください"