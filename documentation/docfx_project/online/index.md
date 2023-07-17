# Online Environment
<br>

> [!REGISTER]
> こちらから参加登録!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br>

 &emsp;本大会での採点は、シミュレータ・自動採点機能を備えたオンライン環境で行われます。  
 &emsp;下記手順を参考に、作成していただいたパッケージ群をオンライン環境にアップロードしてください。アップロード完了後、オンライン環境にてシミュレーションが開始され、結果が提示されます。
## Upload Procedure to Online Environment
1. 動作確認

   &emsp; [LocalEnvironment](../local/index.html)ページを参考に、動作確認をお願いします。  
   &emsp;※`aichallenge_submit`に、作成したパッケージがまとまっているかをご確認ください。
2. ソースコードの圧縮

	 &emsp;下記コマンドにより、`aichallenge_submit`を圧縮してください。dockerディレクトリ内に`aichallenge_submit.tar.gz`が生成されます。
	
	```
	#aichallenge2023-simディレクトリで
	cd docker
	bash create_submit_tar.sh
	```
3.  オンライン環境にアップロード    
	<img src="../images/online/siteImage.png" width="100%">  
	 &emsp;[オンライン環境](https://aichallenge.synesthesias.jp)にアクセスし、手順3で作成した`aichallenge_submit.tar.gz`を「ファイルを選択」からアップロードしてください。アップロードが完了すると、ソースコードのビルド・シミュレーションの順番で実施されます。
	* 正常に終了した場合は、採点完了と表示され、result.jsonがダウンロードできるようになります。また、また距離点・タイムがランキングに掲示されます。
	* 正常にシナリオ実行が終了しても、launchに失敗した等でスコアが出力されていない場合はNo result、チェックポイントを全て通過していない場合はCheckpoint not passedと表示され、いずれの場合も最終的なタイムとしては使われません。
	* ビルドに失敗した場合はBuild errorが表示されます。再度、手順の確認をお願いします。
	* シミュレーターの実行に失敗した場合はSimulator errorと表示されます。この場合サーバーサイドで内部エラーが生じている可能性があるため再度アップロードお願いします。繰り返し表示されてしまう場合はお問合せください。
	* 採点プロセスは一度の提出で5回行われ、結果はその平均によって決定されます。
	* 採点実行中は新たなソースのアップロードはできません。またアップロードできるのは1日3回までで、日本時間0時に回数はリセットされます。
   
4.  結果を確認

　 &emsp;オンライン環境にて評価が終わると、result.jsonがダウンロード可能になります。result.jsonをダウンロードし、結果を確認してください。