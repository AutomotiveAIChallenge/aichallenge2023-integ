# Local Enviroment
　参加者の皆様にはシナリオを遂行するROS2パッケージを作成していただきますが、本リポジトリ内でそのベースとなるサンプルコードとしてautoware/aichallenge_ws/srcに以下のROS2パッケージを提供しております。
  　作成していただいたROS2パッケージは、aichallenge_ws/src/aichallenge_submit以下に配置していただき、下記手順でビルド・実行できるようにしてください。
## Sample Code
### About Sample Code  
* aichallenge_launch
  * 大元のlaunchファイルaichallenge.launch.xmlを含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
* aichallenge_eval
  * スコア算出用のパッケージです。
* aichallenge_score_msgs
  * メッセージ定義を含みます。
* aichallenge_submit
  * このディレクトリの内容は自由に変更していただいて構いません。
  * 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。配布段階で以下のパッケージを含んでいます。
  * aichallenge_submit_launch
    * aichallenge_submit_launch.launch.xmlが大元のlaunchファイルaichallenge.launch.xmlから呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
  * sample_code_cpp
    * サンプルの自動走行実装です。
  * obstacle_stop_planner_custom
    * autoware.universeのobstacle_stop_plannerから障害物を誤検出する問題を解消しています。
* tier4_\*_launch
  * autowareのlaunchファイルをコピーして一部編集したものです。autowareのtier4_\*_launchは削除しているため、こちらを必ずaichallenge_submit内に残すようにしてください。
obstacle_stop_plannerの代わりにobstacle_stop_planner_customを呼び出すように変更してあります。
  
### Build Sample Code
```
# Rockerコンテナ内で
cd /aichallenge/aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```

### Run Sample Code
```
# Rockerコンテナ内で
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml
```

ここまででAutoware側の設定・実行は完了です。セットアップが正常に行われていれば、rvizには点群地図が表示され、自動運転が開始されます。