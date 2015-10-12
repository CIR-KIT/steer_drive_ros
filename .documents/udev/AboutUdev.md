# udevを使ったデバイスの名前固定と権限設定

udevを使えばデバイスの名前を固定できてかつ権限まで設定できるみたいなのでちょっとやってみる備忘録。

参考URL
- http://demura.net/misc/4493.html
- http://mohammedari.blogspot.jp/2013/10/ubuntuurgviewertopurg.html
- http://ubuntuforums.org/showthread.php?t=1265469
- https://docs.oracle.com/cd/E39368_01/e48214/ch07s03.html
- http://wiki.ros.org/hokuyo_node

## デバイス固有の情報を調べる

### デバイスを接続して `udevadm` を実行

ここではiMCS01を例にやってみる。
まず、事前にiMCs01のドライバを`insmod`しておく必要がある。このドライバはPCを再起動するたびに`insmod`する必要がある。
iMCs01のドライバを`insmod`して、デバイスを接続したら、

```bash
udevadm info -a -p $(udevadm info -q path -n /dev/urbtc0)
```

を実行する。すると、次のような結果が得られる。途中で略してある。

```bash
 looking at device '/devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.4/1-1.4:1.0/usbmisc/urbtc0':
    KERNEL=="urbtc0"
    SUBSYSTEM=="usbmisc"
    DRIVER==""

...
```

上記の実行結果から、`idVendor`と`idProduct`を探す。
iMCs01だったら次のような感じでした。
```bash
ATTRS{idVendor}=="1d6b"
ATTRS{idProduct}=="0002"
```

これを各デバイスついて調べる。
3号機の場合だと、iMCs01の他にLRFが3台とArduinoが1台。

## `/etc/udev/rules.d/91-third-robot.rules` というファイルを作成する

`91-third-robot.rules`の最初の数字は読み込まる順番を示しているらしい。
中身は次のような感じ。
LRFの場合は固有の情報が `udevadm` を使って取得できないのでROSの力を使う（強い）。roswikiを参考に。

```
# ttyACM0: Arduino, ttyACM1: LRF(bottom), ttyACM2: LRF(top), ttyACM3: LRF(rear), urbtc0: iMCs01
KERNEL=="ttyACM*  SUBSYSTEMS=="usb" ATTRS{idVendor}=="2341" ATTRS{idProduct}=="0043" NAME="ttyACM0" MODE="0777"

KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0777", GROUP="dialout", PROGRAM="/opt/ros/indigo/env.sh rosrun hokuyo_node getID %N q", SYMLINK+="sensors/hokuyo_%c"

KERNEL=="urbtc*"  ATTRS{idVendor}=="1d6b" ATTRS{idProduct}=="0002" MODE="0777" NAME="urbtc0"
```

このドキュメントと同じ階層にあるファイルをコピーしても良い。
```bash
$ sudo cp <catkin_ws>/src/tc2015_ws/src/TC2015/third_robot/.documents/udev/91-third-robot.udev /etc/udev/rules.d/
```

## restart

```bash
$sudo service udev restart
```

## 挿しなおしてみたりする

権限とかもちゃんと設定できてると思う。
```bash
ls -l /dev/sensors/  
lrwxrwxrwx 1 root root 10 10月 12 19:13 hokuyo_H1101524 -> ../ttyACM2 # top
lrwxrwxrwx 1 root root 10 10月 12 19:12 hokuyo_H1101683 -> ../ttyACM1 # bottom
```
