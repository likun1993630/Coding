## 搜索文件
与搜索相关的命令常用的有 whereis，which，find 和 locate

- whereis 简单快速
```shell
$ whereis who
$ whereis find

who: /usr/bin/who /usr/share/man/man1/who.1.gz
find: /usr/bin/find /usr/share/man/man1/find.1.gz /usr/share/info/find.info.gz
```
> 你会看到 whereis find 找到了三个路径，两个可执行文件路径和一个 man 在线帮助文件所在路径，这个搜索很快，因为它并没有从硬盘中依次查找，而是直接从数据库中查询。whereis 只能搜索二进制文件(-b)，man 帮助文件(-m)和源代码文件(-s)。如果想要获得更全面的搜索结果可以使用 locate 命令。

- locate 快而全

通过“ /var/lib/mlocate/mlocate.db ”数据库查找，不过这个数据库也不是实时更新的，系统会使用定时任务每天自动执行 updatedb 命令更新一次，所以有时候你刚添加的文件，它可能会找不到，需要手动执行一次 updatedb 命令（在我们的环境中必须先执行一次该命令）。它可以用来查找指定目录下的不同文件类型，如查找 /etc 下所有以 sh 开头的文件：
```shell
$ sudo apt-get update
$ sudo apt-get install locate
$ locate /etc/sh

/etc/shadow
/etc/shadow-
/etc/shells
```
> 注意，它不只是在 /bin 目录下查找，还会自动递归子目录进行查找。




