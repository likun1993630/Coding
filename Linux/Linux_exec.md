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


