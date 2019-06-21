## 寻找文件
找出 /etc/ 目录下的所有以 .list 结尾的文件。

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
