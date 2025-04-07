#!/bin/bash

# エラー発生時に処理を停止
set -e

# 引数の検証
if [ $# -lt 2 ]; then
  echo "使用方法: $0 <メールアドレス> <パスワード>"
  exit 1
fi

QT_EMAIL=$1
QT_PASSWORD=$2

# メールアドレスの簡易検証
if [[ ! "$QT_EMAIL" =~ ^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$ ]]; then
  echo "エラー: 有効なメールアドレスを指定してください"
  exit 1
fi

# シークレットディレクトリの作成（必要に応じて）
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

echo "Qt認証情報を使用してインストーラースクリプトを生成しています..."

# qsファイルを生成
cat > qt-installer-nonintaractive.qs << EOF
function Controller() {
    installer.autoRejectMessageBoxes();
    installer.setMessageBoxAutomaticAnswer("installationError", QMessageBox.Retry);
    installer.setMessageBoxAutomaticAnswer("installationErrorWithRetry", QMessageBox.Retry);
    installer.setMessageBoxAutomaticAnswer("DownloadError", QMessageBox.Retry);
    installer.setMessageBoxAutomaticAnswer("archiveDownloadError", QMessageBox.Retry);
    installer.installationFinished.connect(function() {
        gui.clickButton(buttons.NextButton);
    })
}

Controller.prototype.WelcomePageCallback = function() {
    // click delay here because the next button is initially disabled for ~1 second
    gui.clickButton(buttons.NextButton, 7000);
}

Controller.prototype.CredentialsPageCallback = function() {
    var page = gui.pageWidgetByObjectName("CredentialsPage");
    page.loginWidget.EmailLineEdit.setText("$QT_EMAIL");
    page.loginWidget.PasswordLineEdit.setText("$QT_PASSWORD");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.IntroductionPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.TargetDirectoryPageCallback = function()
{
    gui.currentPageWidget().TargetDirectoryLineEdit.setText("/opt/Qt/install");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.PerformInstallationPageCallback = function() {
    gui.clickButton(buttons.CommitButton);
}

Controller.prototype.ComponentSelectionPageCallback = function() {
    function list_packages() {
      var components = installer.components();
      console.log("Available components: " + components.length);
      var packages = ["Packages: "];
      for (var i = 0 ; i < components.length ;i++) {
          packages.push(components[i].name);
      }
      console.log(packages.join(" "));
    }

    list_packages();

    var widget = gui.currentPageWidget();
    widget.deselectAll();
    widget.selectComponent("qt.qt5.5123.gcc_64");

    gui.clickButton(buttons.NextButton);
}

Controller.prototype.LicenseAgreementPageCallback = function() {
    gui.currentPageWidget().AcceptLicenseRadioButton.setChecked(true);
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.StartMenuDirectoryPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.ReadyForInstallationPageCallback = function()
{
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.FinishedPageCallback = function() {
    var checkBoxForm = gui.currentPageWidget().LaunchQtCreatorCheckBoxForm;
    if (checkBoxForm && checkBoxForm.launchQtCreatorCheckBox) {
        checkBoxForm.launchQtCreatorCheckBox.checked = false;
    }
    gui.clickButton(buttons.FinishButton);
}
EOF

# ファイルのパーミッション設定
chmod 600 qt-installer-nonintaractive.qs

echo "インストーラースクリプトが正常に生成されました"
echo "注意: このファイルには認証情報が含まれているため、ビルド後に必ず削除してください"