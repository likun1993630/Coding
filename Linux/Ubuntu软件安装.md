## 有道词典
> 注意要下载deepin版本，不能下载ubuntu版本。

- 官网下载deepin dep包
- 安装
```shell
$ cd Downloads
$ sudo dpkg -i youdao-dict_1.1.0-0-deepin_amd64.deb
$ sudo apt-get -f install
$ source .zshrc
```

可能需要先自己安装依赖
```shell
$ sudo apt-get install python3-pyqt5 -y;sudo apt-get install python3-xlib -y;sudo apt-get install tesseract-ocr -y;sudo apt-get install tesseract-ocr-eng -y;sudo apt-get install tesseract-ocr-chi-sim -y;sudo apt-get install tesseract-ocr-chi-tra -y;sudo apt-get install ttf-wqy-microhei -y;sudo apt-get install python3-pyqt5.qtmultimedia -y;sudo apt-get install python3-pyqt5.qtquick -y;sudo apt-get install python3-pyqt5.qtwebkit -y;sudo apt-get install qtdeclarative5-controls-plugin -y;sudo apt-get install libqt5multimedia5-plugins -y
```


## 蓝牙链接问题

- 原文：
  sudo apt-get install blueman bluez*
  vim /etc/bluetooth/main.conf
  去掉行[Policy]和AutoEnable前的注释
  并把AutoEnable=false,改成AutoEnable=true

  sudo vi /lib/udev/rules.d/50-bluetooth-hci-auto-poweron.rules
  每行都加上 # 开头，其实也可以删除了这个文件。

  重启电脑，命令行输入bluetoothctl

- 实际：
  ```shell
  # zsh不支持*符号匹配
  $ bash
  $ sudo apt-get install blueman bluez*
  ```
  然后使用蓝牙菜单设备进行连接。
  
