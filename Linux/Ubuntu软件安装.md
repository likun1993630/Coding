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
