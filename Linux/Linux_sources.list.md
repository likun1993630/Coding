apt软件包管理系统使用一个私有数据库跟踪列表中软件包的当前状态：已安装、未安 装或可安装。apt-get通过该数据库来确定如何安装用户想用的软件包以及正常运行该软件包所必须的其它关联包。 使用sudo apt-get update来更新数据库列表。这个命令将扫描 /etc/apt/sources.list文件中所指路径（链接）中的软件包列表文件。

## /etc/apt/sources.list文件
/etc/apt/sources.list 文件内容是APT使用的可获得软件包的镜像站点的地址，也就是软件源或者软件源镜像。

另外：
和sources.list功能一样的是/etc/apt/sources.list.d/*.list(*代表一个文件名，只能由字母、数字、下划线、英文句号组成)。sources.list.d目录下的*.list文件为在单独文件中写入源的地址提供了一种方式，通常用来安装第三方的软件。

文件中的各项信息通常按如下格式列出：
- `deb http://host/debian distribution section1 section2 section3`
- `deb-src http://host/debian distribution section1 section2 section3`

- deb
 - 包含的是二进制软件包(deb)， 即我们通常使用的已编译好的软件包；
- deb-src
 - 包含的是源码包(deb-src)，源码包包含源程序编码。
 

一个sources.list 的示例：
```shell
# deb cdrom:[Ubuntu 16.04.6 LTS _Xenial Xerus_ - Release amd64 (20190227)]/ xenial main restricted

# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://de.archive.ubuntu.com/ubuntu/ xenial main restricted
# deb-src http://de.archive.ubuntu.com/ubuntu/ xenial main restricted

## Major bug fix updates produced after the final release of the
## distribution.
deb http://de.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
# deb-src http://de.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
...

```
> 其中`#` 是注释，如果注释掉一个地址，将不从该地址下载软件包
> 修改list文件后，需要运行`sudo apt-get update` 更新。
> 其中的网页链接可以用浏览器打开，类似与一个ftp的文件夹。

## 修改ubuntu的sources.list源
- 首先备份源列表
```shell
$ sudo cp /etc/apt/sources.list /etc/apt/sources.list.save1
```
- 使用编辑器打开sources.list
```shell
$ sudo subl /etc/apt/sources.list
```
- 粘贴源并保存
 - 此处以阿里源为例
```
deb http://mirrors.aliyun.com/ubuntu/ trusty main restricted universe multiverse deb http://mirrors.aliyun.com/ubuntu/ trusty-security main restricted universe multiverse deb http://mirrors.aliyun.com/ubuntu/ trusty-updates main restricted universe multiverse deb http://mirrors.aliyun.com/ubuntu/ trusty-proposed main restricted universe multiverse deb http://mirrors.aliyun.com/ubuntu/ trusty-backports main restricted universe multiverse deb-src http://mirrors.aliyun.com/ubuntu/ trusty main restricted universe multiverse deb-src http://mirrors.aliyun.com/ubuntu/ trusty-security main restricted universe multiverse deb-src http://mirrors.aliyun.com/ubuntu/ trusty-updates main restricted universe multiverse deb-src http://mirrors.aliyun.com/ubuntu/ trusty-proposed main restricted universe multiverse deb-src http://mirrors.aliyun.com/ubuntu/ trusty-backports main restricted universe multiverse
```
- 刷新列表
```shell
$ sudo apt-get update
```
