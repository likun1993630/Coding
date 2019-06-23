## 常见的日志

日志是一个系统管理员，一个运维人员，甚至是开发人员不可或缺的东西，系统用久了偶尔也会出现一些错误，我们需要日志来给系统排错，在一些网络应用服务不能正常工作的时候，我们需要用日志来做问题定位，日志还是过往时间的记录本，我们可以通过它知道我们是否被不明用户登录过等等。

在 Linux 中大部分的发行版都内置使用 syslog 系统日志，那么通过前期的课程我们了解到常见的日志一般存放在 /var/log 中，我们来看看其中有哪些日志

```shell
# 查看系统日志
ll /var/log

-rw-r--r-- 1 root              root  26K 6月  21 23:11 alternatives.log
-rw-r----- 1 root              adm   990 6月  23 13:38 apport.log
-rw-r----- 1 syslog            adm   29K 6月  23 22:53 auth.log
-rw-r--r-- 1 root              root  57K 2月  27 00:57 bootstrap.log
-rw------- 1 root              utmp    0 6月   1 11:06 btmp
....

```
上面的信息可以根据服务对象粗略的将日志分为两类

- 系统日志
- 应用日志

系统日志主要是存放系统内置程序或系统内核之类的日志信息如 alternatives.log 、btmp 等等，应用日志主要是我们装的第三方应用所产生的日志如 tomcat7 、apache2 等等。

系统日志信息：

| 日志名称           | 记录信息                                                                   |
|--------------------|----------------------------------------------------------------------------|
| alternatives.log   | 系统的一些更新替代信息记录                                                 |
| apport.log         | 应用程序崩溃信息记录                                                       |
| apt/history.log    | 使用 apt-get 安装卸载软件的信息记录                                        |
| apt/term.log       | 使用 apt-get 时的具体操作，如 package 的下载、打开等                       |
| auth.log           | 登录认证的信息记录                                                         |
| boot.log           | 系统启动时的程序服务的日志信息                                             |
| btmp               | 错误的信息记录                                                             |
| Consolekit/history | 控制台的信息记录                                                           |
| dist-upgrade       | dist-upgrade 这种更新方式的信息记录                                        |
| dmesg              | 启动时，显示屏幕上内核缓冲信息,与硬件有关的信息                            |
| dpkg.log           | dpkg 命令管理包的日志。                                                    |
| faillog            | 用户登录失败详细信息记录                                                   |
| fontconfig.log     | 与字体配置有关的信息记录                                                   |
| kern.log           | 内核产生的信息记录，在自己修改内核时有很大帮助                             |
| lastlog            | 用户的最近信息记录                                                         |
| wtmp               | 登录信息的记录。wtmp可以找出谁正在进入系统，谁使用命令显示这个文件或信息等 |
| syslog             | 系统信息记录                                                               |


可使用cat,less 命令查看日志内容：

```shell
$ less /var/log/auth.log
$ cat /var/log/auth.log
```

alternatives.log 中的信息：
```
# 可以从中得到的信息有程序作用，日期，命令，成功与否的返回码

update-alternatives 2019-06-09 18:25:46: run with --install /usr/bin/gnome-www-browser gnome-www-browser /usr/bin/firefox 40
update-alternatives 2019-06-09 18:25:46: run with --install /usr/bin/x-www-browser x-www-browser /usr/bin/firefox 40
update-alternatives 2019-06-19 18:01:25: run with --install /usr/lib/kde4/libexec/kdesu kdesu /usr/lib/kde4/libexec/kdesu-distrib/kdesu 50
update-alternatives 2019-06-19 18:01:25: link group kdesu updated to point to /usr/lib/kde4/libexec/kdesu-distrib/kdesu

```

在 apt 文件夹中的日志信息，其中有两个日志文件 history.log 与 term.log，两个日志文件的区别在于 history.log 主要记录了进行了哪个操作，相关的依赖有哪些，而 term.log 则是较为具体的一些操作，主要就是下载包，打开包，安装包等等的细节操作。


其他的日志格式也都类似于之前我们所查看的日志，主要便是时间，操作。而这其中有两个比较特殊的日志，其查看的方式比较与众不同，因为这两个日志并不是 ASCII 文件而是被编码成了二进制文件，所以我们并不能直接使用 less、cat、more 这样的工具来查看，这两个日志文件是 wtmp，lastlog.使用 last 与 lastlog 工具来提取其中的信息.

```shell
$ last wtmp
$ last lastlog
$ lastlog -u likun

wtmp begins Sun Jun  2 02:01:12 2019

wtmp begins Sun Jun  2 02:01:12 2019

用户名           端口     来自             最后登陆时间
likun                                      **从未登录过**
```



