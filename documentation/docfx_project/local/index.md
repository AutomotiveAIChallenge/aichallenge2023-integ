# Local Enviroment
 &emsp;参加者の皆様にはシナリオを遂行するROS2パッケージを作成していただきますが、本リポジトリ内でそのベースとなるサンプルコードとしてaichallenge2023-sim/docker/aichallenge/aichallenge_ws/srcに以下のROS2パッケージを提供しております。
 &emsp;作成していただいたROS2パッケージは、aichallenge_ws/src/aichallenge_submit以下に配置していただき、下記手順でビルド・実行できるようにしてください。
  
## Sample Code
 &emsp;aichallenge2023-sim/docker/aichallenge/aichallenge_ws/src配下の構成を一部示します。
* aichallenge_launch
    * 大元のlaunchファイルaichallenge.launch.xmlを含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
* aichallenge_scoring
    * AI チャレンジの参加者のスコアリングとランキングに必要なタスクを担当します。
* aichallenge_scoring_msgs
    * スコアリング用のmsg
* aichallenge_scoring_result
    * /aichallenge/scoreと/aichallenge/collisionからスコアを算出します。
* aichallenge_submit
    * 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。
    *  aichallenge_submit_launch.launch.xmlが大元のlaunchファイルaichallenge.launch.xmlから呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
    * self_driving_controlloer
        * 自動走行のサンプルです。
    * autoware_launch
        * Autowareの起動構成リポジトリ。ノード構成とそのパラメーターが含まれます。Autowareの動作の変更を行う場合は、まずこちらをご確認ください。

### Docker Image Build
```
#aichallenge2023-simディレクトリで
cd docker
bash build.sh
```

###  Docker Container Run
```
#aichallenge2023-simディレクトリで
cd docker
bash run_container.sh
```

### Code Build
```
# Rockerコンテナ内で
cd /aichallenge
bash build.sh
```

### Sample Code Run
```
# Rockerコンテナ内で
cd /aichallenge
bash run.sh
```


 &emsp;ここまででAutoware側の設定・実行は完了です。セットアップが正常に行われていれば、rvizには点群地図が表示され、自動運転が開始されます。