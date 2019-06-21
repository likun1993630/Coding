## Linux 下软件安装（以Ubuntu 为例）
ubuntu是基于debian的发行版，它使用的是debian的包管理工具dpkg，所以一些操作也适用与debian。

通常 Linux 上的软件安装主要有四种方式：
- 在线安装（APT）
- 从磁盘安装deb软件包 
- 从二进制软件包安装
- 从源代码编译安装

### apt 包管理工具介绍 

APT是Advance Packaging Tool（高级包装工具）的缩写，是Debian及其派生发行版的软件包管理器，APT可以自动下载，配置，安装二进制或者源代码格式的软件包，因此简化了Unix系统上管理软件的过程。APT最早被设计成dpkg的前端，用来处理deb格式的软件包。现在经过APT-RPM组织修改，APT已经可以安装在支持RPM的系统管理RPM包。这个包管理器包含以 apt- 开头的多个工具，如 apt-get apt-cache apt-cdrom 等，在Debian系列的发行版中使用。


安装一个软件，名字叫做 w3m(w3m是一个命令行的简易网页浏览器)，那么输入如下命令：
```shell
$ sudo apt-get install w3m

Reading package lists... Done
Building dependency tree       
Reading state information... Done
Suggested packages:
  w3m-img libsixel-bin w3m-el cmigemo
The following NEW packages will be installed:
  w3m
0 upgraded, 1 newly installed, 0 to remove and 0 not upgraded.
Need to get 898 kB of archives.
After this operation, 2,234 kB of additional disk space will be used.
Get:1 http://de.archive.ubuntu.com/ubuntu xenial-updates/universe amd64 w3m amd64 0.5.3-26ubuntu0.2 [898 kB]
Fetched 898 kB in 0s (2,258 kB/s)
Selecting previously unselected package w3m.
(Reading database ... 416465 files and directories currently installed.)
Preparing to unpack .../w3m_0.5.3-26ubuntu0.2_amd64.deb ...
Unpacking w3m (0.5.3-26ubuntu0.2) ...
Processing triggers for mime-support (3.59ubuntu1) ...
Processing triggers for man-db (2.7.5-1) ...
Setting up w3m (0.5.3-26ubuntu0.2) ...
```
> 意:如果你在安装一个软件之后，无法立即使用Tab键补全这个命令，你可以尝试先执行source ~/.zshrc，然后你就可以使用补全操作。
```shell
$ source .zshrc
```

使用w3m打开一个网页
```shell

$ w3m www.baidu.com
```

当你在执行安装操作时，首先apt-get 工具会在本地的一个数据库中搜索关于 w3m 软件的相关信息，并根据这些信息在相关的服务器上下载软件安装，这里大家可能会一个疑问：既然是在线安装软件，为啥会在本地的数据库中搜索？要解释这个问题就得提到几个名词了：
- 软件源镜像服务器
- 软件源

我们需要定期从服务器上下载一个软件包列表，使用 sudo apt-get update 命令来保持本地的软件包列表是最新的（有时你也需要手动执行这个操作，比如更换了软件源），而这个表里会有软件依赖信息的记录，对于软件依赖，我举个例子：我们安装 w3m 软件的时候，而这个软件需要 libgc1c2 这个软件包才能正常工作，这个时候 apt-get 在安装软件的时候会一并替我们安装了，以保证 w3m 能正常的工作。

> 可以这样理解，本地数据库用来查找需要下载那些依赖软件，然后去软件源把所有需要的软件下载下来然后一块安装。 软件源决定了下载的快慢。


### apt-get 
apt-get 是用于处理 apt包的公用程序集，我们可以用它来在线安装、卸载和升级软件包等，下面列出一些apt-get包含的常用的一些工具：

| 工具         | 说明                                                                                   |
|--------------|----------------------------------------------------------------------------------------|
| install      | 其后加上软件包名，用于安装一个软件包                                                   |
| update       | 从软件源镜像服务器上下载/更新用于更新本地软件源的软件包列表                            |
| upgrade      | 升级本地可更新的全部软件包，但存在依赖问题时将不会升级，通常会在更新之前执行一次update |
| dist-upgrade | 解决依赖关系并升级(存在一定危险性)                                                     |
| remove       | 移除已安装的软件包，包括与被移除软件包有依赖关系的软件包，但不包含软件包的配置文件     |
| autoremove   | 移除之前被其他软件包依赖，但现在不再被使用的软件包                                     |
| purge        | 与remove相同，但会完全移除软件包，包含其配置文件                                       |
| clean        | 移除下载到本地的软件包，默认保存在/var/cache/apt/archives/                   |
| autoclean    | 移除已安装的软件的旧版本软件包                                                         |

下面是一些apt-get常用的参数：

| 参数               | 说明    |
|--------------------|------------------------------------------------------------|
| -y                 | 自动回应是否安装软件包的选项，在一些自动化安装脚本中使用这个参数将十分有用   |
| -s                 | 模拟安装  |
| -q                 | 静默安装方式，指定多个q或者-q=#,#表示数字，用于设定静默级别，这在你不想要在安装软件包时屏幕输出过多时很有用 |
| -f                 | 修复损坏的依赖关系   |
| -d                 | 只下载不安装 |
| --reinstall        | 重新安装已经安装但可能存在问题的软件包 |
| --install-suggests | 同时安装APT给出的建议安装的软件包  |


当一些错误的配置导致软件无法正常工作时，重新安装：
```shell
$ sudo apt-get --reinstall install w3m
```
> 有时候你需要同时安装多个软件包，你还可以使用正则表达式匹配软件包名进行批量安装。

软件升级：

```shell
# 更新软件源
$ sudo apt-get update
# 升级没有依赖问题的软件包
$ sudo apt-get upgrade
# 升级并解决依赖关系
$ sudo apt-get dist-upgrade
```

卸载软件：
```shell
# 保留配置文件
$ sudo apt-get remove w3m

# 不保留配置文件的移除
$ sudo apt-get purge w3m
# 或者 sudo apt-get --purge remove

# 移除不再需要的被依赖的软件包
$ sudo apt-get autoremove
```

软件搜索:

当自己刚知道了一个软件，想下载使用，需要确认软件仓库里面有没有，就需要用到搜索功能了，命令如下：

```shell
$ sudo apt-cache search softname1 softname2 softname3……
```
> apt-cache 命令则是针对本地数据进行相关操作的工具，search 顾名思义在本地的数据库中寻找有关 softname1 softname2 …… 相关软件的信息。

```shell
$ sudo apt-cache search w3m
```

其他关于apt的内容参考：
[APT HOWTO](https://www.debian.org/doc/manuals/apt-howto/index.zh-cn.html#contents)

### 使用dpkg（Debian Package）
使用 dpkg 从本地磁盘安装 deb 软件包

> dpkg 是 Debian 软件包管理器的基础,被用于安装、卸载和供给和 .deb 软件包相关的信息。
> dpkg 本身是一个底层的工具。上层的工具，像是 APT，被用于从远程获取软件包以及处理复杂的软件包关系。
> 以deb形式打包的软件包，就需要使用dpkg命令来安装。

dpkg常用参数介绍：

| 参数 | 说明                                            |
|------|-------------------------------------------------|
| -i   | 安装指定deb包                                   |
| -R   | 后面加上目录名，用于安装该目录下的所有deb安装包 |
| -r   | remove，移除某个已安装的软件包                  |
| -I   | 显示deb包文件的信息                             |
| -s   | 显示已安装软件的信息                            |
| -S   | 搜索已安装的软件包                              |
| -L   | 显示已安装软件包的目录信息                      |

使用dpkg安装deb软件包

```shell
# 使用下载模式，下载deb到本地
$ sudo apt-get -d install emacs24

# 查看本地文件夹内容：
$ ls /var/cache/apt/archives 


emacs24_24.5+1-6ubuntu1.1_amd64.deb
```

```shell
# 
将deb拷贝到 /home/likun 目录下，并使用dpkg安装：
$ cp /var/cache/apt/archives/emacs24_24.5+1-6ubuntu1.1_amd64.deb ~

# 安装之前参看deb包的信息
$ sudo dpkg -I emacs24_24.5+1-6ubuntu1.1_amd64.deb

# 使用dpkg安装，注意会出错误，因为有些依赖没能提前安装
$ sudo dpkg -i emacs24_24.5+1-6ubuntu1.1_amd64.deb


Selecting previously unselected package emacs24.
(Reading database ... 411934 files and directories currently installed.)
Preparing to unpack emacs24_24.5+1-6ubuntu1.1_amd64.deb ...
Unpacking emacs24 (24.5+1-6ubuntu1.1) ...
dpkg: dependency problems prevent configuration of emacs24:
 emacs24 depends on emacs24-bin-common (= 24.5+1-6ubuntu1.1); however:
  Package emacs24-bin-common is not installed.
 emacs24 depends on libm17n-0 (>= 1.6.1); however:
  Package libm17n-0 is not installed.
 emacs24 depends on libotf0 (>= 0.9.11); however:
  Package libotf0 is not installed.

dpkg: error processing package emacs24 (--install):
 dependency problems - leaving unconfigured
Processing triggers for man-db (2.7.5-1) ...
Processing triggers for gnome-menus (3.13.3-6ubuntu3.1) ...
Processing triggers for desktop-file-utils (0.22-1ubuntu5.2) ...
Processing triggers for bamfdaemon (0.5.3~bzr0+16.04.20180209-0ubuntu1) ...
Rebuilding /usr/share/applications/bamf-2.index...
Processing triggers for mime-support (3.59ubuntu1) ...
Errors were encountered while processing:
 emacs24
```

要解决这个问题，就要用到apt-get了，使用它的-f参数了，修复依赖关系的安装：

```shell
$ sudo apt-get update
$ sudo apt-get -f install
```
> 这样貌似只能修复apt库中包含的依赖包

查看已安装软件包的安装目录：

```shell
# 使用dpkg -L查看deb包目录信息
$ sudo dpkg -L emacs24

/.
/usr
/usr/share
/usr/share/lintian
/usr/share/lintian/overrides
/usr/share/lintian/overrides/emacs24
/usr/share/man
/usr/share/man/man1
....

```


### 从二进制包安装（可执行的软件包）
需要做的只是将从网络上下载的二进制包解压后放到合适的目录，然后将包含可执行的主程序文件的目录添加进PATH环境变量。

下载leanote软件二进制包，是zip文件。 `leanote-desktop-linux-x64-v2.6.2.zip`

使用unzip命令解压后目录如下：

```
leanote
├── content_shell.pak
├── icudtl.dat
├── Leanote # 可执行文件，终端运行即可启动软件
├── leanote.png
├── libffmpeg.so
├── libnode.so
├── LICENSE
├── LICENSES.chromium.html
├── locales
├── __MACOSX
├── natives_blob.bin
├── resources
├── snapshot_blob.bin
└── version
```
将该文件夹移动到一个存放柔软的地方如： /home/likun/Apps/

添加环境变量：

