# Evaluation Enviroment

## Outline of Execution Flow in an Online Environment
　スコアの算出にあたっては、オンライン評価環境のwebページよりパッケージaichallenge_submitのみを提出していただき、自動採点を行います。 提出後、オンライン評価環境ではevaluation/以下のスクリプトを使って下記の手順で評価されます。
1.  aichallenge_submitの配置  
　アップロードしていただいたaichallenge_submit.tar.gzはevaluation/以下に配置されます。
  
2. docker build  
　evaluation/build.shが実行され、evaluation/Dockerfileで定義されるdockerイメージが作成されます。このイメージの作成手順は下記の通りです。
    1. 提出いただいたaichallenge_submit.tar.gzを/aichallenge/aichallenge_ws/src/aichallenge_submitへ展開  
    2. rosdep installとcolcon buildの実行    
          
3. シミュレーション実行  
　オンライン評価環境でsimulatorが立ち上がり、シミュレーションが開始されます。  
　コンテナ内ではevaluation/main.bashの実行によって、以下が行われます。
    1. ROS2ノード群の起動
    2. シナリオの開始
    evaluation/run.shで実行した場合、evaluation/output以下に結果(score.json)が保存されます。
    

## Procedures for Submitting Source Code
1. ソースコードを圧縮する
　aichallenge_submit内のソースコードを圧縮します。
　
   ```
   cd evaluation
   sh create_submit_tar.sh
   ```
   evaluation/aichallenge_submit.tar.gzに圧縮済みのファイルが生成されていることを確認してください。
     
2. evaluation/ でdocker内での自動実行ができることを確認する
　オンライン評価環境にアップロードする前に、ローカル環境を使いオンライン環境と同様のDockerコンテナ内でビルド・実行ができることを以下の手順で確認してください。  
    まず、以下のファイルがevaluation/以下に配置されていることを確認してください。
    * aichallenge_submit.tar.gz    
次に、作成いただいたaichallenge_submitを含むdockerイメージをビルドしてください。
   ```
   sh build.sh
   ```

      ビルドが完了したら、run.shによってdockerコンテナを立ち上げ採点のフローを実行してください。
   
   ```
   sh run.sh
   ```
   最後に、evaluation/output/score.jsonに出力されるスコアを確認してください。
3. オンライン評価環境webページよりソースをアップロードする
[webページ]()にログイン後画面の指示に従って(1)で作成したaichallenge_submit.tar.gzをアップロードしてください。  
  
    アップロードが終了すると、ソースのビルド・シミュレーションの実行が順番に行われます。 
    * 正常に終了した場合はScoring completeと表示され、配布シナリオ・評価用シナリオそれぞれのタイムが表示されます。最後にアップロードした評価シナリオのタイムが、ランキングにて最終タイムとして使われます。
    * 正常にシナリオ実行が終了しても、launchに失敗した等でスコアが出力されていない場合はNo result、チェックポイントを全て通過していない場合はCheckpoint not passedと表示され、いずれの場合も最終的なタイムとしては使われません。
    * ビルドに失敗した場合はBuild errorが表示されます。(1),(2)の手順に従ってDocker imageのビルドができることを再度ご確認ください。
    * シミュレーターの実行に失敗した場合はSimulator errorと表示されます。この場合サーバーサイドで内部エラーが生じている可能性があるため再度アップロードお願いします。繰り返し表示されてしまう場合はお問合せください。
    * 採点プロセスは一度の提出で5回行われ、結果はその平均によって決定されます。
    
    なお、採点実行中は新たなソースのアップロードはできません。またアップロードできるのは1日3回までで、日本時間0時にリセットされます。