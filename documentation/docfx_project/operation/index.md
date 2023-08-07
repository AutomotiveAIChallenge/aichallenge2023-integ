# Operation

<br>

&emsp;このページでは、 AutowareをインストールしたノートPCで実際の車両を動かす手順を解説します。

## Vehicle Interfaceの有効化
本選で使用するゴルフカートのVehicle Interfaceの実行ファイルが`~/vi_ws` の中に含まれています。このVehicle Interfaceを使用する方法を説明します。


1. シンボリックリンクの適用 
以下のコマンドを実行し、 シンボリックリンクを適用する。

```
$ ln -snf /home/autoware/<開発に使用しているレポジトリ> /home/autoware/aichallenge2023final-test
```

3. Vehicle Interface の起動確認
`scripts/run.sh` を実行後、別のターミナルで`ros2 node list |grep g30` を実行し、以下のように、Vehicle Interfaceのノード名が出力されていることを確認する。

```
/g30esli/socket_can_receiver
/g30esli/socket_can_sender
/g30esli_interface
/g30esli_interface_awiv_adapt_receiver
/g30esli_interface_awiv_adapt_sender
```

## 車両HWとの接続方法
LiDARとCameraデータを取得するための方法を説明します。

### LiDARの接続
ToDo: Velodyneの接続設定を選択する という内容に変更、 スクショかなにか挿入

### CAN インターフェースの接続
1. 車両から出ている Type-A USBコネクタをPCに接続する。
ターミナルにてチェック用のスクリプトを実行する。
    ```
    $ ./can_config.sh # 成功ならば出力はCAN configuration done
    $ ifconfig # txqueuelenが500000になっていれば成功 ※ToDo can_config.sh内に判定を入れる
    ```

### 接続確認
1. ターミナル上で `~/hw_connectivity_check.sh` を実行し、 LiDAR、 CAN Interfaceの接続状況を確認する。
    ```
    $ ~/hw_connectivity_check.sh #ToDo: 診断をスクリプトに含める
    ```

## 車両IDの設定．
Autowareを実行するターミナルにて、 環境変数 `VEHICLE_ID` を設定する。
```
$ export VEHICLE_ID=3 # 3号車の場合。 VEHICLE_IDは競技当日メンターから提示されます。
```

## Autoware起動〜自動運転開始の操作
※車両を動かす前に，必ず [安全に関する注意点](#安全に関する注意点)を一読ください．

1. 下記コマンドでAutowareを起動する．
    ```
    $ source /opt/ros/humble/setup.bash #.bashrc記載の場合は不要
    $ cd ~/<ToDo: フォルダ名確認>
    $ source install/setup.bash #.bashrc記載の場合は不要
    $ cd scripts
    $ ./run_autoware_on_vehicle.sh #ToDo: can_config.shの内容を含める
    ```

2. 自己位置推定を開始する。
    - 2D pose estimate をrVizから入力する。

3. ゴール地点を指定する．
    - ターミナルを開き、`~/<ToDo ディレクトリ名>/scripts/set_goal.sh`を実行してゴール地点を設定する．

4. ToDo: エンゲージまでの手順を追記


## 車両を動かすフロー
### 自動運転発進までの手順
1. セーフティドライバー: Golf Cartを自動モードに設定する．
2. メンター: 競技参加者に "Webコントローラーを起動．Vehicle engageとAutoware engageのdisengageを押下する" 指示を出す．
3. Autowareオペレーター(参加者): Webコントローラーを起動．Vehicle engageとAutoware engageのdisengageを押下．
4. メンター: Autoware オペレーターに "Vehicle engageを押下する" 指示を出す．
5. Autowareオペレーター(参加者): Vehicle engageのengageを押下．
6. セーフティドライバー: ステアの挙動から，正常にVehicle engageされたことを確認．
7. メンター: "RVizに切り替え，Autoware engageを押下する" 指示を出す．
8. Autowareオペレーター: RVizに切り替え，Autoware engageボタンを押下．

### オーバーライド（ドライバーが車両を停止させたとき）が発生した場合のフロー
1. Disengage する ※ToDo: どうやるか書く

## 安全に関する注意点
- 競技車両の不具合により、Vehicle Engageへの遷移に失敗することがあります。セーフティドライバーがVehicle Engage遷移失敗を判断します. 本事象が発生した場合は、Autoware Engage と Vehicle EngageをFalseにした後、自動運転発進までの手順をやり直してください．
- 自動走行時，車両に同乗しない参加者は、 歩道上で待機してください．経路上や車両周囲は立ち入り禁止です。
- 自動走行時は必ず上部の手すりを掴んでください。
- 自動運転に使用するノートPCをバンドで固定してください。 キーボードが打ちづらい場合は貸出しているUSB無線キーボードを使用してください。
