# AIChallenge2023 

## Introduction
&emsp;本サイトでは、自動運転AIチャレンジ2023インテグレーション大会の予選大会に参加する方向けに、大会ルールや環境構築方法など、大会に参加するために必要な情報をまとめています。  
  
&emsp;昨年度行った、自動運転AIチャレンジ2022シミュレーション大会と同様に、本大会では自動運転ソフトウェアAutoware.universeと自動運転シミュレータAWSIMを使用します。[Setupページ](../setup)記載の手順に沿って環境を構築し、大会へご参加ください。
  
## About Competition
&emsp;皆様には下記の流れに沿って、本大会に取り組んでいただきます。  
1. 与えられたシナリオをクリアできるようなソフトウェアを開発  
2. ローカル環境で手順1で作成したソフトウェアを検証  
3. 検証が完了したソフトウェアをオンライン環境にアップロード  
4. オンライン上でシミュレーションが実施されて、タイムを計測   
    ※最後にアップロードされたソースコードのシミュレーション結果のタイムに基づいて順位を決定します。（オンライン環境へのご案内は後日行います）  
&emsp;大会ルールの詳細は[Ruleページ](../rule)をご参照ください。

## About Autoware
&emsp; AutowareとはROS2を使用したオープンソースの自動運転ソフトウェアです。LiDARやカメラなどからデータを取得するセンシング機能、センシングデータを組み合わせて車両の位置を推定するLocalization機能などがモジュールとして存在し、それらが相互に連携することで、自動運転を実現しています。本ソフトウェアは、日本国内の公道での実証実験も行われています。  
&emsp; 本大会ではAutowareの中でも研究・開発向けディストリビューションであるAutoware.universeを使用します。その他のディストリビューションやAutowareのこれまでの開発の流れについては、[こちら](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/difference-from-ai-and-auto/)を御覧ください。
  ![autoware](/images/intro/autoware.png)
  
## About AWSIM
 &emsp;AWSIMはUnity上で動作するオープンソースの自動運転用シミュレータになります。ROS2 nativeに対応していること、WindowsやUbuntuに対応していることから、誰でも簡単に自動運転アルゴリズムのシミュレーションを行うことが可能です。
  &emsp;AutowareでAWSIMを活用した場合、AWSIMからのセンシングデータをAutowareのノードがSubscribeし、受け取ったデータを各モジュールで処理した結果である車両制御情報をAWSIMにPublishすることで、AWSIM上の車両を制御します。
 ![awsim](/images/intro/awsim.png)
 
## Related Documentations
 * [自動運転Aiチャレンジ公式HP](https://www.jsae.or.jp/jaaic/)
 * [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)
 *  [AWSIM](https://github.com/tier4/AWSIM)
 
## Page Links
 * [Introduction](../intro)  予選大会について
 * [Setup](../setup)  環境構築手順について
 * [Rule](../rule) ルールについて
 * [About Local Enviroment](../local) ローカル開発環境について
 * [About Evaluation Enviroment](../evaluation) オンライン評価環境について
 * [Other](../other) 問い合わせ・参加者同士の情報共有方法など
 
 