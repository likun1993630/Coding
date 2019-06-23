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

## 点号（.）
等价于 source 命令

**bash 中的 source 命令用于在当前 bash 环境下读取并执行 FileName.sh 中的命令。**

```shell
$ source test.sh
$ . test.sh
Hello World
Hello World
```

## 引号 
- 双引号（")
  "STRING" 将会阻止（解释）STRING中大部分特殊的字符。后面的实验会详细说明。
- 单引号（'）
  'STRING' 将会阻止STRING中所有特殊字符的解释，这是一种比使用"更强烈的形式。后面的实验会详细说明。
  
```shell
$ HOME='world'
$ echo $HOME
$ echo '$HOME'
$ echo "$HOME"

world
$HOME
world
```
> 同样是$HOME,单引号会直接认为是字符，而双引号认为是一个变量

## 斜线和反斜线 
### 斜线（/）
文件名路径分隔符。分隔文件名不同的部分（如/home/bozo/projects/Makefile）。也可以用来作为除法算术操作符。注意在linux中表示路径的时候，许多个/跟一个/是一样的。`/home/shiyanlou`等同于`////home///shiyanlou`

### 反斜线（\）
一种对单字符的引用机制。\X 将会“转义”字符X。这等价于"X"，也等价于'X'。\ 通常用来转义双引号（"）和单引号（'），这样双引号和单引号就不会被解释成特殊含义了。

符号 说明
- \n 表示新的一行
- \r 表示回车
- \t 表示水平制表符
- \v 表示垂直制表符
- \b 表示后退符
- \a 表示"alert"(蜂鸣或者闪烁)
- \0xx 转换为八进制的ASCII码, 等价于0xx
- \" 表示引号字面的意思

转义符也提供续行功能，也就是编写多行命令的功能。

每一个单独行都包含一个不同的命令，但是每行结尾的转义符都会转义换行符，这样下一行会与上一行一起形成一个命令序列。

## 反引号（`） 
命令替换

反引号中的命令会优先执行，如：
```shell
$ cp `mkdir back` test.sh back
$ ls
```
> 先创建了 back 目录，然后复制 test.sh 到 back 目录

## 冒号（:） 
### 空命令

等价于“NOP”（no op，一个什么也不干的命令）。也可以被认为与shell的内建命令true作用相同。“:”命令是一个bash的内建命令，它的退出码（exit status）是（0）。

如：
```shell
#!/bin/bash

while :
do
    echo "endless loop"
done
```

等价于
```shell
#!/bin/bash

while true
do
    echo "endless loop"
done
```
可以在 if/then 中作占位符：
```shell
#!/bin/bash

condition=5

if [ $condition -gt 0 ] #gt表示greater than，也就是大于，同样有-lt（小于），-eq（等于） 
then :   # 什么都不做，退出分支
else
    echo "$condition"
fi
```
### 变量扩展/子串替换
在与>重定向操作符结合使用时，将会把一个文件清空，但是并不会修改这个文件的权限。如果之前这个文件并不存在，那么就创建这个文件。
```shell
$ : > test.sh   # 文件“test.sh”现在被清空了
# 与 cat /dev/null > test.sh 的作用相同
# 然而,这并不会产生一个新的进程, 因为“:”是一个内建命令
```

在与>>重定向操作符结合使用时，将不会对预先存在的目标文件 (: >> target_file)产生任何影响。如果这个文件之前并不存在，那么就创建它。

也可能用来作为注释行，但不推荐这么做。使用 # 来注释的话，将关闭剩余行的错误检查，所以可以在注释行中写任何东西。然而，使用 : 的话将不会这样。如：
```shell
$ : This is a comment that generates an error,( if [ $x -eq 3] )
```

":"还用来在 /etc/passwd 和 $PATH 变量中做分隔符，如：
```shell
$ echo $PATH

/usr/local/bin:/bin:/usr/bin:/usr/X11R6/bin:/sbin:/usr/sbin:/usr/games
```
## 问号（?） 

测试操作符

在一个双括号结构中，? 就是C语言的三元操作符，如：
```shell
$ vim test.sh
```
输入如下代码，并保存：
```shell
#!/bin/bash

a=10
(( t=a<50?8:9 ))
echo $t
```
运行测试
```shell
$ bash test.sh
8
```

## 美元符号（$）

```shell
$ vim test.sh
```
写入文件
```shell
#!/bin/bash
var1=5
var2=23skidoo
echo $var1     # 5
echo $var2     # 23skidoo
```
运行测试
```shell
$ bash test.sh
5
23skidoo
```
## 小括号（( )） 
### 命令组
在括号中的命令列表，将会作为一个子 shell 来运行。

在括号中的变量，由于是在子shell中，所以对于脚本剩下的部分是不可用的。父进程，也就是脚本本身，将不能够读取在子进程中创建的变量，也就是在子shell 中创建的变量。如：
```shell
$ vim test20.sh
```
输入代码：
```shell
#!/bin/bash
a=123
( a=321; )
echo "$a" #a的值为123而不是321，因为括号将判断为局部变量
```
运行代码：
```shell
$ bash test20.sh

a = 123
```
> 在圆括号中 a 变量，更像是一个局部变量。

### 初始化数组

创建数组
```shell
$ vim test21.sh
```
输入代码：
```shell
#!/bin/bash

arr=(1 4 5 7 9 21)
echo ${arr[3]} # get a value of arr
```
运行代码：
```shell
$ bash test21.sh

7
```

## 大括号（{ }）
### 文件名扩展
复制 t.txt 的内容到 t.back 中
```shell
$ vim test22.sh
```
输入代码：
```shell
#!/bin/bash

if [ ! -w 't.txt' ];
then
    touch t.txt
fi
echo 'test text' >> t.txt
cp t.{txt,back}
```
运行代码：
```shell
$ bash test22.sh
```
查看运行结果：
```shell
$ ls
$ cat t.txt
$ cat t.back
```
> 注意： 在大括号中，不允许有空白，除非这个空白被引用或转义。

### 代码块

代码块，又被称为内部组，这个结构事实上创建了一个匿名函数（一个没有名字的函数）。然而，与“标准”函数不同的是，在其中声明的变量，对于脚本其他部分的代码来说还是可见的。
```shell
$ vim test23.sh
```
输入代码：
```shell
#!/bin/bash
a=123
{ a=321; }
echo "a = $a"
```

运行代码：
```shell
$ bash test23.sh

a = 321
```
> 变量 a 的值被更改了。

## 中括号（[ ]）
### 条件测试

条件测试表达式放在[ ]中。下列练习中的-lt (less than)表示小于号。
```shell
$ vim test24.sh
```
输入代码：
```shell
#!/bin/bash

a=5
if [ $a -lt 10 ]
then
    echo "a: $a"
else
    echo 'a>10'
fi
```
运行代码：
```shell
$ bash test24.sh

a: 5
```
> 双中括号（[[ ]]）也用作条件测试（判断），后面的实验会详细讲解。

### 数组元素
在一个array结构的上下文中，中括号用来引用数组中每个元素的编号。
```shell
$ vim test25.sh
```
输入代码：
```shell
#!/bin/bash

arr=(12 22 32)
arr[0]=10
echo ${arr[0]}
```
运行代码：
```shell
$ bash test25.sh

10
```

## 尖括号（< 和 >） 

重定向

- test.sh > filename：重定向test.sh的输出到文件 filename 中。如果 filename 存在的话，那么将会被覆盖。
- test.sh &> filename：重定向 test.sh 的 stdout（标准输出）和 stderr（标准错误）到 filename 中。
- test.sh >&2：重定向 test.sh 的 stdout 到 stderr 中。
- test.sh >> filename：把 test.sh 的输出追加到文件 filename 中。如果filename 不存在的话，将会被创建。


## 竖线（|） 
管道

分析前边命令的输出，并将输出作为后边命令的输入。这是一种产生命令链的好方法。
```shell
$ vim test26.sh
```
输入代码：
```shell
#!/bin/bash

tr 'a-z' 'A-Z'
exit 0
```
现在让我们输送ls -l的输出到一个脚本中：
```shell
$ chmod 755 test26.sh
$ ls -l | ./test26.sh
```
> 输出的内容均变为了大写字母。

## 破折号（-） 
### 选项，前缀
在所有的命令内如果想使用选项参数的话,前边都要加上“-”。
```shell
$ vim test27.sh
```
输入代码：
```shell
#!/bin/bash

a=5
b=5
if [ "$a" -eq "$b" ]
then
    echo "a is equal to b."
fi
```
运行代码：
```shell
$ bash test27.sh

a is equal to b.
```
### 用于重定向stdin或stdout

下面脚本用于备份最后24小时当前目录下所有修改的文件.
```shell
$ vim test28.sh
```
输入代码：
```shell
#!/bin/bash

BACKUPFILE=backup-$(date +%m-%d-%Y)
# 在备份文件中嵌入时间.
archive=${1:-$BACKUPFILE}
#  如果在命令行中没有指定备份文件的文件名,
#  那么将默认使用"backup-MM-DD-YYYY.tar.gz".

tar cvf - `find . -mtime -1 -type f -print` > $archive.tar
gzip $archive.tar
echo "Directory $PWD backed up in archive file \"$archive.tar.gz\"."

exit 0
```
运行代码：
```shell
$ bash test28.sh
$ ls
```

## 浪号（~） 
目录

~ 表示 home 目录。

