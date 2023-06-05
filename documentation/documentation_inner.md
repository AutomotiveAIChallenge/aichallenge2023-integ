# 開発者(内部)向けドキュメント

## ローカルでのドキュメント確認手順
1. [docfx](https://dotnet.github.io/docfx/)のインストール
   1. docfx2.67.1以上のバージョンでない場合は動作しない可能性があるので注意
2. ビルド、サーバー起動
```
cd aichallenge2023-sim/documentation
docfx --serve
```
1. Webサイトへのアクセス
   1. ブラウザでhttp://localhost:8080/を開くとドキュメントが確認できる。