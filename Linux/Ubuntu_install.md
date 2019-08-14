1. 安装对exfat格式的支持
```shell
sudo apt-get install exfat-utils
```
2. 安装 git
```shell
sudo apt-get install git
```
3. Surface pro4 内核

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

4. 主题

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

5. 安装 zsh 和 配置文件oh my zsh
```shell
sudo apt-get install zsh
zsh --version
wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh
echo $SHELL
chsh -s $(which zsh)
```
注销/切换一次用户就切换为zsh


6.安装Clion

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


7. 安装 CMake

参考
https://blog.csdn.net/lj402159806/article/details/76408597/

```shell
sudo apt install cmake
```
最好参考链接进行一个简单项目的测试


8. 安装Python3

9. 安装 ROS Kinetic
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





&&&&&&&&&&&&&&&&&&&&&&&&&&&&&不安装内核&&&&&&&&&&&&&&&&&&&&&&&&&&&&
…………………………………………………………………………………………………………………………………………………………………………………………………………………

10.Surface pro4 wifi 崩溃问题
https://askubuntu.com/questions/791976/installing-ubuntu-on-surface-pro-4
https://github.com/Ubuntu-SurfacePro-4/deriver-config

https://github.com/jakeday/linux-surface

https://grenangen.se/node/86

https://askubuntu.com/questions/953711/ubuntu-16-04-on-surface-book-wireless-internet-disconnects-randomly-until-rest



