# crane_x7_examples

設計製作論3,グループ8のCRANE-X7を用いたシステムに関するリポジトリです。このパッケージではcrane_x7の前方特定範囲内にある空き缶を拾って投げ捨てる事が出来ます。

## システムの起動方法

CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合

実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。
このパッケージはプログラムに追記しない限りgazebo上でカラー画像、点群を取得する事は出来ません。それを行いたい場合はlocate_estimate.cpp内のカラー画像、点群を受け取るトピックに適切なものを指定してください。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### 実機を使う場合

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

ケーブルの接続ポート名はデフォルトで`/dev/ttyUSB0`です。
別のポート名(例: /dev/ttyUSB1)を使う場合は次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false port:=/dev/ttyUSB1
```

### Gazeboを使う場合

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

## Run "poisute" motion 

`demo.launch`を実行している状態で弊班が作ったロボットによる缶の"ポイ捨て"動作を実行できます。

### find_and_hold.launch

実機動作に対応しています。

crane_x7の前方特定範囲にピンクの折り紙を巻いた缶がある場合それを拾ってポイ捨てをします。無かった場合はテーブル全体を薙ぎ払う動作を行います。

#### 実機を使う場合

右の図のようにピンクの折り紙を巻いた空き缶を用意します。
<img src=https://github.com/8group-robotdesign3/crane_x7_ros_modified_by_group8/blob/master/crane_x7_examples/akikan.jpg height=300px />

ロボットにポイ捨てをさせる場合は以下の図の青い四角形の範囲内に空き缶を設置します。
青い範囲外かつ以下の図の黒い半円の範囲内に空き缶を置いた場合薙ぎ払う動作を行います。

<img src=https://github.com/8group-robotdesign3/crane_x7_ros_modified_by_group8/blob/master/crane_x7_examples/haitizu.png width=500px />

次のコマンドで実行します。

```sh
roslaunch crane_x7_examples find_and_hold.launch
```
