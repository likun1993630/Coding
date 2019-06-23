## 寻找文件1
找出 /etc/ 目录下的所有以 .list 结尾的文件。

解法：

- locate

```shell
$ sudo locate /etc/\*.list 

/etc/apt/sources.list
/etc/apt/sources.list.d/github_git-lfs.list
/etc/apt/sources.list.d/jonathonf-ubuntu-python-3_6-xenial.list
/etc/apt/sources.list.d/noobslab-ubuntu-icons-xenial.list
/etc/apt/sources.list.d/noobslab-ubuntu-themes-xenial.list
/etc/apt/sources.list.d/ros-latest.list
/etc/apt/sources.list.d/sublime-text.list
/etc/gnome/defaults.list
/etc/ros/rosdep/sources.list.d/10-fssim-dependencies.list
/etc/ros/rosdep/sources.list.d/20-default.list
```

- find

```shell
$ sudo find /etc/ -name \*.list

/etc/gnome/defaults.list
/etc/ros/rosdep/sources.list.d/10-fssim-dependencies.list
/etc/ros/rosdep/sources.list.d/20-default.list
/etc/apt/sources.list
/etc/apt/sources.list.d/ros-latest.list
/etc/apt/sources.list.d/jonathonf-ubuntu-python-3_6-xenial.list
/etc/apt/sources.list.d/sublime-text.list
/etc/apt/sources.list.d/noobslab-ubuntu-themes-xenial.list
/etc/apt/sources.list.d/github_git-lfs.list
/etc/apt/sources.list.d/noobslab-ubuntu-icons-xenial.list
```

## 寻找文件2
介绍

有一个非常重要的文件（sources.list）但是你忘了它在哪了，你依稀记得它在 /etc/ 目录下，现在要你把这个文件找出来，然后设置成自己（shiyanlou 用户）可以访问，但是其他用户并不能访问。

目标
- 找到 sources.list 文件
- 把文件所有者改为自己（shiyanlou）
- 把权限修改为仅仅只有自己可读可写

提示语
- find
- chmod
- chown
- sudo

> 注意：如果是实验楼的海外用户，由于环境差异可能会找到两个 sources.list 文件，不需要修改 shiyanlou 目录下的 sources.list，因为这个文件是从实验环境外部挂载到环境中的，是无法修改的。

解法：
```shell
$ sudo find /etc -name sources.list
$ mkdir test
$ cp /etc/apt/sources.list test
$ cd test
$ ls -l sources.list
$ sudo chown lilei sources.list
$ ls -l sources.list
$ sudo chown likun sources.list
$ ls -l sources.list
$ chmod 600 sources.list
$ ls -l sources.list

/etc/apt/sources.list
-rw-rw-r-- 1 likun likun 2904 Jun 21 11:20 sources.list
-rw-rw-r-- 1 lilei likun 2904 Jun 21 11:20 sources.list
-rw-rw-r-- 1 likun likun 2904 Jun 21 11:20 sources.list
-rw------- 1 likun likun 2904 Jun 21 11:20 sources.list

```

## 文件夹大小排序
找出当前目录下面占用最大的前十个文件。

```shell
$ du -s * | sort -nr | head

# 后10个
$ du -s * | sort -nr | tail


20488000	VirtualBox VMs
4891044	Downloads
1081588	clion-2018.3.4
819868	Udacity
426896	下载
245760	Apps
67056	amz
23556	selfie_carolocup2019
16148	abc
7748	selfie_carolocup2019 (复件)
```

## 文本处理
资源下载
```shell
$ cd /home/likun
$ wget http://labfile.oss.aliyuncs.com/courses/1/data1
```
data1 文件里记录是一些命令的操作记录，现在需要你从里面找出出现频率次数前3的命令并保存在 /home/likun/result。

目标：

处理文本文件 /home/shiyanlou/data1
将结果写入 /home/shiyanlou/result
结果包含三行内容，每行内容都是出现的次数和命令名称，如“100 ls”

结果：
```shell
cat data1 |cut -c 8-|sort|uniq -dc|sort -rn -k1 |head -3 > ~/result
```

