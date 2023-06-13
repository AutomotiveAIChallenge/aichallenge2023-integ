# aichallenge2023-sim
[![.github/workflows/document.yml](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/actions/workflows/document.yml/badge.svg?branch=feature%2Fen)](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/actions/workflows/document.yml)

![aichallenge2023](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/assets/113989589/9b56b775-0628-4e26-af86-331a5c13a6bb)　　
　　
  
&emsp; 本ページは、2023年度の7月から10月にかけて実施される自動運転AIチャレンジ2023インテグレーション大会に関するページです。　　　　

&emsp; 参加者の皆様には、Autoware.Universe をベースとした、自動運転ソフトウェアを開発していただきます。開発したソフトウェアをインテグレーションした自動運転車で、テーマに基づいた3つの課題をクリアし、 短い時間でコースを完走することが目標です。  
&emsp;本大会は予選と本戦に分かれています。予選ではデジタルツインの自動運転シミュレーター AWSIM上で自動運転車を走行させ、本選では実際の車両で取得したデータをもとに、車両のチューニング、開発を行った後、リアルな環境に用意された予選と同様のコースに挑みます。

![schedule_2023](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/assets/113989589/58e75a25-b823-4b20-9282-885a21527af8)

## ドキュメント
&emsp; 下記ページに大会の詳細や環境構築方法などがまとめられています。ご参照の上、大会へご参加ください
- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge2023-sim/index.html)
- [English Page](https://automotiveaichallenge.github.io/aichallenge2023-sim/en/index.html)

## ファイル構成
本リポジトリに含まれるファイルの構成について記載します。

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
