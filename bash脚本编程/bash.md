# 什么是Bash 
Bash（GNU Bourne-Again Shell）是一个为GNU计划编写的Unix shell，它是许多Linux平台默认使用的shell。

shell是一个命令解释器，是介于操作系统内核与用户之间的一个绝缘层。准确地说，它也是能力很强的计算机语言，被称为解释性语言或脚本语言。它可以通过将系统调用、公共程序、工具和编译过的二进制程序”粘合“在一起来建立应用，这是大多数脚本语言的共同特征，所以有时候脚本语言又叫做“胶水语言”

事实上，所有的UNIX命令和工具再加上公共程序，对于shell脚本来说，都是可调用的。Shell脚本对于管理系统任务和其它的重复工作的例程来说，表现的非常好，根本不需要那些华而不实的成熟紧凑的编译型程序语言。

# 初步练习
## Hello World
```shell
$ vim hello.sh
```

使用vim编辑hello.sh ，输入如下代码并保存：
```
#!/bin/bash
# This is a comment
echo Hello World
```

> vim中插入按i

> 保存并退出换行按esc然后输入:wq再enter

> #! 是说明 hello 这个文件的类型，有点类似于 Windows 系统下用不同文件后缀来表示不同文件类型的意思（但不相同）。

> Linux 系统根据 "#!" 及该字符串后面的信息确定该文件的类型，可以通过 man magic命令 及 /usr/share/magic 文件来了解这方面的更多内容。

> 在 BASH 中 第一行的 "#!" 及后面的 /bin/bash 就表明该文件是一个 BASH 程序，需要由 /bin 目录下的 bash 程序来解释执行。BASH 这个程序一般是存放在 /bin 目录下，如果你的 Linux 系统比较特别，bash 也有可能被存放在 /sbin 、/usr/local/bin 、/usr/bin 、/usr/sbin 或 /usr/local/sbin 这样的目录下；如果还找不到，你可以用 locate bash ,find / -name bash 2>/dev/null 或 whereis bash 这三个命令找出 bash 所在的位置；如果仍然找不到，那你可能需要自己动手安装一个 BASH 软件包了。

> 第二行的 "# This is a ..." 就是 BASH 程序的注释，在 BASH 程序中从“#”号（注意：后面紧接着是“!”号的除外）开始到行尾的部分均被看作是程序的注释。

> 第三行的 echo 语句的功能是把 echo 后面的字符串输出到标准输出中去。由于 echo 后跟的是 "Hello World" 这个字符串，因此 "Hello World"这个字串就被显示在控制台终端的屏幕上了。需要注意的是 BASH 中的绝大多数语句结尾处都没有分号。

运行Bash脚本的方式：
```shell
# 使用shell来执行
$ sh hello.sh
# 使用bash来执行
$ bash hello.sh
使用.来执行
$ . ./hello.sh
使用source来执行
$ source hello.sh
还可以赋予脚本所有者执行权限，允许该用户执行该脚本
$ chmod u+rx hello.sh
$  ./hello.sh
```

## 使用重定向

比如我们想要保存刚刚的hello world为一个文本，那么该怎么办呢？

> 这个符号是重定向,执行以下代码，就会在当前目录下生成一个my.txt。打开看看有没有hello world

```shell
 #!/bin/bash
 echo "Hello World" > my.txt
```

## 使用脚本清除/var/log下的log文件

首先我们看一看/var/log/wtmp里面有啥东西
```shell
$ cat /var/log/wtmp
```

这个文件中记录了系统的一些信息，现在我们需要写一个脚本把里面的东西清空，但是保留文件
```shell
# 创建并编辑脚本
$ vim cleanlogs.sh
```
> /dev/null这个东西可以理解为一个黑洞，里面是空的（可以用cat命令看一看）

cleanlogs.sh内容：
```shell
#!/bin/bash

# 初始化一个变量
LOG_DIR=/var/log
cd $LOG_DIR
# 用空内容覆盖wtmp文件的内容
cat /dev/null > wtmp
echo "Logs cleaned up."
exit
```
运行脚本前，先使用 sudo chmod +x cleanlogs.sh 授予脚本执行权限，然后再看看 /var/log/wtmp 文件内是否有内容。运行此脚本后，文件的内容将被清除。
```shell
$ sudo chmod +x cleanlogs.sh
# 使用sudo命令调用管理员权限才能执行成功
$ sudo ./cleanlogs.sh
$ cat /var/log/wtmp
```

> 由于脚本中含有对系统日志文件内容的清除操作，这要求要有管理员权限.不然会报permission denied错误

> #!/bin/bash这一行是表示使用/bin/bash作为脚本的解释器，这行要放在脚本的行首并且不要省略

> 脚本正文中以#号开头的行都是注释语句，这些行在脚本的实际执行过程中不会被执行。这些注释语句能方便我们在脚本中做一些注释或标记，让脚本更具可读性。

注意：
> `sudo cat /dev/null > /var/log/wtmp`会提示权限不够，为什么呢？因为sudo只能让cat命令以sudo的权限执行，而对于>这个符号并没有sudo的权限，我们可以使用:

> `sudo sh -c "cat /dev/null > /var/log/wtmp " `让整个命令都具有sudo的权限执行


# bash特殊字符
## 注释 #：

行首以 # 开头(除#!之外)的是注释。#!是用于指定当前脚本的解释器，我们这里为bash，且应该指明完整路径，所以为/bin/bash

当然，在echo中转义的 # 是不能作为注释的：
```shell
$vim test.sh
```
输入如下代码，并保存。（中文为注释，不需要输入）
```shell
#!/bin/bash

echo "The # here does not begin a comment."
echo 'The # here does not begin a comment.'
echo The \# here does not begin a comment.
echo The # 这里开始一个注释
echo $(( 2#101011 ))     # 数制转换（使用二进制表示），不是一个注释，双括号表示对于数字的处理
```
> 上面的脚本说明了如何使用echo打印出一段字符串和变量内容

## 分号（;） 
### 命令分隔符
使用分号（;）可以在同一行上写两个或两个以上的命令。

```shell
$ vim test2.sh
```
输入如下代码，并保存。

```shell
 #!/bin/bash
 echo hello; echo there
 filename=ttt.sh
 if [ -e "$filename" ]; then    # 注意: "if"和"then"需要分隔，-e用于判断文件是否存在
     echo "File $filename exists."; cp $filename $filename.bak
 else
     echo "File $filename not found."; touch $filename
 fi; echo "File test complete."
```

执行脚本
```shell
$ bash test2.sh
```
> 上面脚本使用了一个if分支判断一个文件是否存在，如果文件存在打印相关信息并将该文件备份；如果不存在打印相关信息并创建一个新的文件。最后将输出"测试完成"。

### 终止case选项（双分号）

使用双分号（;;）可以终止case选项。
```shell
$ vim test3.sh
```

输入如下代码，并保存。
```shell
#!/bin/bash

varname=b

case "$varname" in
    [a-z]) echo "abc";;
    [0-9]) echo "123";;
esac
```

执行脚本，查看输出
```shell
$ bash test3.sh
abc
```
> 上面脚本使用case语句，首先创建了一个变量初始化为b,然后使用case语句判断该变量的范围，并打印相关信息。

### 点号（.）
等价于 source 命令

**bash 中的 source 命令用于在当前 bash 环境下读取并执行 FileName.sh 中的命令。**
