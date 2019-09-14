## 修改root用户密码
```shell
sudo passwd root
# 此后就可以使用su 来切换到root用户
```

## 使用 .sh 文件快速安装软件

```shell
# home目录下创建一个 install.sh 文件
# 然后将shell 命令拷贝进去
# 使用sh 命令执行 .sh 文件：
sh install.sh
```

## 安装对exfat格式的支持
```shell
sudo apt-get install exfat-utils
```
## 安装 git
```shell
sudo apt-get install git
```
## Surface pro4 内核

https://github.com/jakeday/linux-surface

```shell
# 终端输入：
sudo apt install git curl wget sed
git clone --depth 1 https://github.com/jakeday/linux-surface.git ~/linux-surface
cd ~/linux-surface
sudo sh setup.sh
# 重启 Reboot on installed kernel.
reboot
```

## 主题
参考：
https://blog.csdn.net/White_Idiot/article/details/78973575
https://www.linuxidc.com/Linux/2017-10/147431.htm

```shell
sudo apt-get install unity-tweak-tool 

sudo add-apt-repository ppa:noobslab/themes
sudo apt-get update
sudo apt-get install flatabulous-theme

sudo add-apt-repository ppa:noobslab/icons
sudo apt-get update
sudo apt-get install ultra-flat-icons
```
安装完成后，打开unity-tweak-tool软件，修改主题和图标：
进入Theme，修改为Flatabulous
图标，修改为ultra-flat

苹果主题
https://blog.csdn.net/liudsl/article/details/80145254
苹果工具栏
https://www.jianshu.com/p/bd7b12a1c071

## 安装 zsh 和 配置文件oh my zsh
```shell
sudo apt-get install zsh
zsh --version
wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh
echo $SHELL
chsh -s $(which zsh)
```
注销/切换一次用户就切换为zsh


## 安装Clion
https://blog.csdn.net/qq_32599479/article/details/81088837
https://blog.csdn.net/u011068702/article/details/55190353/

官网下载压缩包
https://www.jetbrains.com/clion/
```shell
# 打开终端
cd ~/下载
tar -xzvf CLion-2018.3.2.tar.gz
cd clion-2018.3.2/bin
./clion.sh
```
UI安装界面逐步安装


## 安装 CMake
参考
https://blog.csdn.net/lj402159806/article/details/76408597/
```shell
sudo apt install cmake
```
最好参考链接进行一个简单项目的测试


## 安装Python3

## 安装 ROS Kinetic
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 或:
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

运行小海龟历程：
```shell
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

zsh 环境变量设置
```shell
sudo gedit ~/.zshrc 
#将source /opt/ros/kinetic/setup.zsh加入其中即可 
source /opt/ros/kinetic/setup.zsh 
#原来在bash中的形式是source /opt/ros/kinetic/setup.bash
```
参考：
https://cloud.tencent.com/developer/news/62201


软件源：
https://www.cnblogs.com/EasonJim/p/7119156.html
https://blog.csdn.net/maizousidemao/article/details/79127695


## 安装 Sublime Text 3

```shell
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text

# 完成后既可以通过 subl 启动
```

## 安装Typora

https://www.typora.io/#linux

```shell
# or run:
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BA300B7755AFCFAE
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add -

# add Typora's repository
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update

# install typora
sudo apt-get install typora
```
## SSD 速度测试

```shell
# 首先在home目录下建立一个文件test
touch ~/test
# 向test文件写入8G的数据，并输出速度和时间
# dd命令将/dev/zero中的无限输入写到test文件中
time dd if=/dev/zero of=~/test bs=8k count=1000000

```

```
记录了1000000+0 的读入
记录了1000000+0 的写出
8192000000 bytes (8,2 GB, 7,6 GiB) copied, 19,0835 s, 429 MB/s

real	0m19.090s
user	0m0.448s
sys	0m7.231s

```

```shell
# 测试纯读速度，我们还是利用dd命令
time dd if=~/test of=/dev/null bs=8k count=1000000
```
```
记录了1000000+0 的读入
记录了1000000+0 的写出
8192000000 bytes (8,2 GB, 7,6 GiB) copied, 15,0281 s, 545 MB/s

real	0m15.034s
user	0m0.626s
sys	0m4.520s
```

```shell
# 测试读写速度
touch ~/test1
time dd if=~/test of=~/test1 bs=8k count=1000000
```
```
记录了1000000+0 的读入
记录了1000000+0 的写出
8192000000 bytes (8,2 GB, 7,6 GiB) copied, 46,0595 s, 178 MB/s

real	0m46.061s
user	0m0.409s
sys	0m10.356s
```

## SSD 优化
https://blog.csdn.net/hzwwpgmwy/article/details/80313272

## swap优化
swappiness的值的大小对如何使用swap分区是有着很大的联系的。swappiness=0的时候表示最大限度使用物理内存，然后才是 swap空间，swappiness＝100的时候表示积极的使用swap分区，并且把内存上的数据及时的搬运到swap空间里面。linux的基本默认设置为60
```shell
# 查看系统当前swappiness值
cat /proc/sys/vm/swappiness

# 设置swappiness值
su
echo "vm.swappiness=1" >> /etc/sysctl.conf

# 注销重新登录之后就被修改了
```

## 修改系统home文件夹内自带文件夹位置

在home下新建 System文件夹，将如下七个文件夹剪切到System下

```
System
├── Desktop
├── Documents
├── Download
├── Music
├── Pictures
├── Public
├── Templates
└── Videos

```

修改相关配置文件：

```shell
subl .config/user-dirs.dirs
# 将内容更改为
```

```
XDG_DESKTOP_DIR="$HOME/System/Desktop"
XDG_DOWNLOAD_DIR="$HOME/System/Download"
XDG_TEMPLATES_DIR="$HOME/System/Templates"
XDG_PUBLICSHARE_DIR="$HOME/System/Public"
XDG_DOCUMENTS_DIR="$HOME/System/Documents"
XDG_MUSIC_DIR="$HOME/System/Music"
XDG_PICTURES_DIR="$HOME/System/Pictures"
XDG_VIDEOS_DIR="$HOME/System/Videos"
```

可能出现的问题：

需要修改蓝牙文件接收路径，`sudo blueman-services`，修改路径，然后重启，注意要重启，注销不行。



## Surface pro4 wifi 崩溃问题

不安装第三方内核的情况下

https://askubuntu.com/questions/791976/installing-ubuntu-on-surface-pro-4

https://github.com/Ubuntu-SurfacePro-4/deriver-config

https://github.com/jakeday/linux-surface

https://grenangen.se/node/86

https://askubuntu.com/questions/953711/ubuntu-16-04-on-surface-book-wireless-internet-disconnects-randomly-until-rest



