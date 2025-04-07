## How to Use
Qtのインストールにはアカウント作成が必要となる。
[Qt公式サイト](https://login.qt.io/login)でアカウント登録を予め行っておくこと。
`qt-installer-nonintaractive.qs`の中の以下の箇所を、登録ユーザのメールアドレス・パスワードに置き換える。

qt-installer-nonintaractive.qs
```javascript
（中略）
Controller.prototype.CredentialsPageCallback = function() {
    var page = gui.pageWidgetByObjectName("CredentialsPage");
    page.loginWidget.EmailLineEdit.setText("Your Email");
    page.loginWidget.PasswordLineEdit.setText("Your Password");
    gui.clickButton(buttons.NextButton);
}
（中略）
```

Dockerイメージをビルドする。かなり時間がかかるので気長に待つ。
```bash
cd <this packgage>
docker build ./ -t ib2_simulator
```

生成したDockerイメージ`ib2_simulator`を使ってコンテナを生成する。
```bash
cd <this packgage>
docker compose up
```

## 役に立ったサイト
-[ROS Docker イメージで発生した GPG error の解消方法](https://zuqqhi2.com/ros-docker-image-gpg-error)
- [dind(docker-in-docker)とdood(docker-outside-of-docker)でコンテナを料理する](https://qiita.com/t_katsumura/items/d5be6afadd6ec6545a9d)
- [【ubuntu】Dockerでsystemctlを使えるようにする](https://zenn.dev/ippe1/articles/327f2b1ed423cb)
- [stack overflow: Ubuntu 16.04 Compile VLC for Android Failed](https://stackoverflow.com/questions/54904765/ubuntu-16-04-compile-vlc-for-android-failed)
- 