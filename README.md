# aichallenge2023-sim
[![.github/workflows/document.yml](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/actions/workflows/document.yml/badge.svg?branch=feature%2Fen)](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/actions/workflows/document.yml)

## ドキュメント
- [pagesへのリンク追記]()

## ファイル構成
本リポジトリに含まれるファイルの構成について記載します。

詳細は[リンク追記]()をご参照ください。

### autoware/aichallenge_ws/src
* aichallenge_launch
  * 大元のlaunchファイルaichallenge.launch.xmlを含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
* aichallenge_eval(**WIP**)
  * スコア算出用のパッケージです。
* aichallenge_score_msgs(**WIP**)
  * メッセージ定義を含みます。
* aichallenge_submit(**WIP**)
  * このディレクトリの内容は自由に変更していただいて構いません。
  * 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。配布段階で以下のパッケージを含んでいます。
  * aichallenge_submit_launch
    * aichallenge_submit_launch.launch.xmlが大元のlaunchファイルaichallenge.launch.xmlから呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
  * sample_code_cpp
    * サンプルの自動走行実装です。
  * autoware_launch
    * [autoware_launchのawsim-stableブランチ](https://github.com/autowarefoundation/autoware_launch/tree/awsim-stable)をコピーしたものです。dockerイメージ内のautoware_launchは削除しているため、こちらを必ずaichallenge_submit内に残すようにしてください。

### autoware/mapfiles
autowareで使用される地図ファイル(pcd, osm)が配置されています。

### documentation
参加者向けのドキュメントが配置されています。

### evaluation
オンライン採点環境での評価手順をローカルで再現するためのファイルが配置されています。
