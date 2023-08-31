# FAQ

<br>

## ROS FAQ

<br>

### pythonでパッケージを作成すると実行時 no module named * のerrorが起こる
[setup.pyにsubmoduleが追加されているか確認してください。](https://zenn.dev/tasada038/articles/5d8ba66aa34b85#setup.py%E3%81%ABsubmodules%E3%81%A8%E3%81%97%E3%81%A6%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E3%82%92%E8%BF%BD%E5%8A%A0%E3%81%99%E3%82%8B)

### `$ ros2 topic list` が表示されない
- あなたのマシンの `ROS_DOMAIN_ID` が一致していることを確認してください。（`ROS_DOMAIN_ID` を設定していない方は問題ないです）
- `ROS2` がソースされていることを確認してください。

<br>

### WindowsのAWSIMとUbuntuのAutowareを使用しており、`$ ros2 topic list` が表示されない
- Windows Firewallでの通信を許可してください。
- `ros2 daemon stop` と `ros2 daemon start` を実行して、不要なプロセスが残っていないか確認し、再起動してください。

<br>

### No path found in Rviz
- あなたのマップデータが正しいか確認してください。PointCloud、VectorMapが含まれます。

<br>

### AWSIM and Autoware network is unstable
localでテストする際、すべてのterminalで `ROS_LOCALHOST_ONLY=1` に設定すると通信速度が向上します。
今回の大会では、PC2台構成のWindows+Linux、Linux+Linux、PC1台でLinuxのみの構成も考慮しています。以下の設定を参照してください。
- 評価環境の[こちら](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/blob/main/docker/evaluation/main.bash)のファイルで `ROS_LOCALHOST_ONLY=0`
- コンテナ側の[こちら](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/blob/main/docker/Dockerfile)のファイルで `ROS_LOCALHOST_ONLY=0`

マシンの性能や通信速度が十分でない場合、以下のようにlocalhostのみでの実行に変更することもできます。
- ROSをlocalhost-onlyに設定します。`.bashrc` に以下の行を追加してください。注意: OSの起動後、ターミナルの起動時にパスワードが要求され、初回には `sudo ip link set lo multicast on` が必要です。

  ```bash
  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  if [ ! -e /tmp/cycloneDDS_configured ]; then
      sudo sysctl -w net.core.rmem_max=2147483647
      sudo ip link set lo multicast on
      touch /tmp/cycloneDDS_configured
  fi
注意：
- 一度上記のように.bashrcに書き込んで変更したことを忘れると常に適用されてしまうことになるため、`echo $ROS_LOCALHOST_ONLY`で確認するなど必ず変更点は追ってください。
- `ROS_LOCALHOST_ONLY=1`と`ROS_LOCALHOST_ONLY=0`が混在しているとcontainer間の通信ができません。
- `ROS_LOCALHOST_ONLY`が実行ファイルに記載されていることには注意してください。

<br>

### Launched Autoware is not stable

Autowareが起動するまでの待機時間を設定してみてください。

https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/issues/31

```
<timer period="150.0">
     <include file="$(find-pkg-share self_driving_controller)/launch/self_driving_controller.launch.xml" />
</timer>
```

<br>

### Unable to Launch Rocker

https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/issues/21#issuecomment-1637851299


<br>

## Setup FAQ

<br>

### Rviz Black Screen when running container

https://github.com/ros2/rviz/issues/948

```
add-apt-repository ppa:kisak/kisak-mesa
apt update
apt upgrade
```

<br>

### AWSIM ends with coredump
AWSIMを起動するだけでcoredumpで終了する場合、GPUのメモリが不足している可能性があります。

Nvidia-smiでGPUメモリの利用率が限界に達していないか確認してください。

GPUのメモリは11GB以上を推奨しています。

<br>

## Other FAQ

tier4:AWSIMのトラブルシューティングも参照できます。

https://github.com/tier4/AWSIM/blob/main/docs/DeveloperGuide/TroubleShooting/index.md