# crane_x7_examples

設計製作論3,グループ8のCRANE-X7を用いたシステムに関するリポジトリです。

## システムの起動方法

CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合

実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。

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

`demo.launch`を実行している状態で弊班が作ったロボットによるペットボトルキャップの"ポイ捨て"動作を実行できます。

### group6.launch

実機動作,シミュレーター共に対応しています。

crane_x7が目の前に落ちているペットボトルキャップを拾ってゴミ箱の前でポイ捨てをするか迷う動作を行った後に後ろに投げ捨てます。

#### 実機を使う場合

以下の二つの図の様にロボットの正面方向前方200mmの所にペットボトルキャップを1つ置き、ゴミ箱をロボットの正面方向前方400mm(ペットボトルキャップから200mm)の所に配置します。

<img src=https://github.com/8group-robotdesign3/crane_x7_ros/blob/master/crane_x7_examples/uekara.png width=500px />

<img src=https://github.com/8group-robotdesign3/crane_x7_ros/blob/master/crane_x7_examples/yokokara.png width=500px />

次のコマンドで実行します。

```sh
roslaunch crane_x7_examples group6.launch
```
