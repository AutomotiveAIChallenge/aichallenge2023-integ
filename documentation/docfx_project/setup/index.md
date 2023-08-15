# SetUp

<br>

<!-- > [!REGISTER]
> こちらから参加登録!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br> -->

## Minimum Hardware Requirements
本大会で使用していただくPCの動作環境として以下を推奨しております。

* OS: Ubuntu 22.04
* CPU: Intel Corei7 (8 cores) or higher
* GPU: NVIDIA Geforce RTX 3080 (VRAM 12 GB) or higher
* Memory: 32 GB or more
* Storage: SSD 30 GB or higher

上記のスペックを満たすPCをご用意できない方は、下記の「PC2台で参加する方向け」のスペックをご参照ください。
#### 2台のPCを使用する方向け
#### Autoware PC
* OS: Ubuntu 22.04
* CPU: Intel Corei7 (8 cores) or higher
* GPU: NVIDIA Geforce GTX 1080 or higher
* Memory: 16 GB or higher
* Storage: SSD 10 GB or higher
* 詳細は[こちら](https://autowarefoundation.github.io/autoware-documentation/main/installation/)

#### AWSIM PC
* OS: Ubuntu 22.04 or Windows 10/11
* CPU: Intel Corei7 (6 cores and 12 threads) or higher
* GPU: NVIDIA Geforce RTX 2080 Ti or higher
* 詳細は[こちら](https://tier4.github.io/AWSIM/)

※Autoware動作PCとAWSIM動作PCは、同じネットワーク内に配置してください。
配置できていれば、基本的には追加設定をすることなく、PC間のトピック通信は可能です。万が一、トピック通信ができなかった場合はファイアーウォールの解除、もしくはルールの見直しをお願いします。
  
    
## Environment Setup
### AWSIM(Ubuntu)
#### 事前準備
* Nvidiaドライバのインストール
  1. リポジトリの追加
  ```
  sudo add-apt-repository ppa:graphics-drivers/ppa
  ```
  2. パッケージリストの更新
  ```
  sudo apt update
  ```
  3. インストール
  ```
  sudo ubuntu-drivers autoinstall
  ```
  4. 再起動の後、下記コマンドを実行し、インストールできていることを確認
  ```
  nvidia-smi
  ```
  ![nvidia-smi](../images/setup/nvidia-smi.png)
 
 * Vulkunのインストール
    1. パッケージリストの更新
    ```
    sudo apt update
    ```
    2. libvulkan1をインストール
    ```
    sudo apt install libvulkan1
    ```
 * コースの準備
   1. [GoogleDrive](https://drive.google.com/drive/folders/1zONmvBjqMzveemkZmNdd4icbpwnDYvTq?usp=sharing)から最新の`AWSIM_AIChallenge_Ubuntu_v*.*.zip`をダウンロードし、解凍
   2. パーミッションを図のように変更    
   ![パーミッション変更の様子](../images/setup/permmision.png)  
   3. ファイルをダブルクリックで起動
   4. 下記のような画面が表示されることを確認
      ![awsim](../images/setup/awsim.png)

### Dockerの事前準備  
下記のインストールをお願いします。
  * [docker](https://docs.docker.com/engine/install/ubuntu/)
  * [rocker](https://github.com/osrf/rocker) 
     * Dockerコンテナ内のRviz、rqtなどのGUIを使用するために用います。
  * [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
  * [git lfs](https://packagecloud.io/github/git-lfs/install)
  * [ROS2](https://docs.ros.org/en/humble/index.html) (動画確認済バージョン: Humble)
  
* Dockerイメージの準備・起動 〜 Autowareの準備
   1. Dockerイメージを入手
    ```
   docker pull ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda:v1
    ```
    ※上記の方法では長時間かかってしまう方・タイムアウトしてしまう方↓  
　[こちら](https://drive.google.com/file/d/1mOEpiN36UPe70NqiibloDcd_ewgMr_5P/view?usp=sharing)に、イメージをtarにまとめたものを置きましたので、下記コマンドよりご利用ください
   ```
   docker load < autoware-universe-cuda_v1.tar.gz
   ```
    2. 大会用データのダウンロード
    ```
    sudo apt install -y git-lfs
    git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2023-sim
    ```
    3. 大会用dockerイメージのビルド
    ```
    cd ./aichallenge2023-sim/docker
    bash build.sh
    ```
    4. 大会用dockerコンテナの起動
    ```
    bash run_container.sh
    ```

#### Dockerコンテナ内でのAWSIM起動
DockerコンテナからAWSIMを起動したい場合は、Dockerイメージの準備手順(前述)に従ってDockerイメージを導入した後、以下の手順で行ってください。
  1. `aichallenge2023-sim/docker/aichallenge`内に大会用AWSIM実行ファイルを展開(以下、`aichallenge2023-sim/docker/aichallenge/AWSIM/AWSIM.x86_64`に配置されているものとします。)
  2. rockerを起動
   新たにterminalを開いて`docker image ls`で以下のようなimageが存在していることを確認してください。
   ```
   aichallenge-train        latest                            f5f05f758f55   2 weeks ago      14.9GB
   ```
   確認ができたら以下のコマンドでrockerを起動してください。
   ```
    cd ./aichallenge2023-sim/docker
    bash run_container.sh
   ```
   新たに開いたterminalで`docker container ls` で以下のようにdocker が存在していることを確認してください。
   ```
   CONTAINER ID   IMAGE          COMMAND       CREATED          STATUS          PORTS     NAMES
   fdbe7cb05782   1f3d763bc501   "/bin/bash"   15 minutes ago   Up 15 minutes             elegant_hellman
   ```
  3. コンテナ内で以下を実行
   ```
    sudo ip link set multicast on lo
    source /autoware/install/setup.bash
    /aichallenge/AWSIM/AWSIM.x86_64
   ```

### AWSIM(Windows)
  1. [GoogleDrive](https://drive.google.com/drive/folders/1p-_rZLDVncssgYTwjBmLKMyGQxOKHV5Q?usp=sharing)から最新の`AWSIM_AIChallenge_Windows_v*.*.zip`をダウンロードし解凍
  2. ファイルをダブルクリックで起動
  3. 下記のような画面が表示されることを確認
    ![awsim](../images/setup/awsim.png)

### 地図データ(pcd, osm)のコピー

![mapfiles](../images/setup/mapfiles.png)

地図データはAWSIMの圧縮ファイル内に格納されています。`AWSIM_Data/StreamingAssets/kashiwanoha2023_integ`に配置されているosmファイルとpcdファイルを`aichallenge2023-sim/docker/aichallenge/mapfile`にコピーして、ファイル構成が以下になるように配置してください。
```
aichallenge2023-sim
└ docker
 └ aichallenge
  └ mapfile
   ├ .gitkeep
   ├ lanelet2_map.osm
   └ pointcloud_map.pcd
```

### Autoware      
 * Autowareの動作確認  
   AWSIMを用いて、Autowareの動作確認を行う方法を記します。
   1. AWSIMを起動
   2. Autowareを起動
   ```
   # Rockerコンテナ内で
   sudo ip link set multicast on lo
   cd /aichallenge
   bash build.sh
   source aichallenge_ws/install/setup.bash
   ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=golfcart sensor_model:=awsim_sensor_kit map_path:=/aichallenge/mapfile
   ```
   3. 下記のような画面(Rviz2)が表示されることを確認  
   ![autoware1](../images/setup/autoware1.png)   
     
   4. 自己位置推定ができていることを確認。正しく推定できていなければ、タブにある2D Pose Estimateを選択し、実際の車両の位置をドラッグで指定  
    ![autoware2](../images/setup/autoware2.png)   
            
   5. タブにある2D Goal Poseを選択し、ゴールポジションをドラッグで指定。画像のように、ルートが表示されている かつ `Routing`が`UNSET`から`SET`に変わっていることを確認（指定してから少し時間がかかります）  
     ![autoware3](../images/setup/autoware3.png)         
       
   6. `OperationMode`の`AUTO`ボタンを押下し、自動運転が開始されることを確認  
     ![autoware4](../images/setup/autoware4.png)   
        
