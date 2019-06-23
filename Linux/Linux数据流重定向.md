## 数据流重定向 

```shell
$ echo 'hello world' > redirect 
$ echo 'www.baidu.com' >> redirect
$ cat redirect

hello world
www.baidu.com
```

### 简单的重定向 
Linux 默认提供了三个特殊设备，用于终端的显示和输出，分别为stdin（标准输入,对应于你在终端的输入），stdout（标准输出，对应于终端的输出），stderr（标准错误输出，对应于终端的输出）。


| 文件描述符 | 设备文件    | 说明     |
|------------|-------------|----------|
| 0          | /dev/stdin  | 标准输入 |
| 1          | /dev/stdout | 标准输出 |
| 2          | /dev/stderr | 标准错误 |

> 文件描述符：文件描述符在形式上是一个非负整数。实际上，它是一个索引值，指向内核为每一个进程所维护的该进程打开文件的记录表。当程序打开一个现有文件或者创建一个新文件时，内核向进程返回一个文件描述符。在程序设计中，一些涉及底层的程序编写往往会围绕着文件描述符展开。但是文件描述符这一概念往往只适用于 UNIX、Linux 这样的操作系统。

- cat 命令连续输入（按ctrl c 退出）(默认使用终端的标准输入作为命令的输入和标准输出作为命令的输出)
  ```shell
  $ cat
  ```

- cat的连续输出（heredoc方式）重定向到一个文件
  ```shell
  $ mkdir Documents
  $ cat > Documents/test.c <<EOF
  #include <stdio.h>

  int main()
  {
      printf("hello world\n");
      return 0;
  }

  EOF
  ```
- cat将一个文件作为命令的输入，标准输出作为命令的输出
  ```shell
  $ cat Documents/test.c
  ```
  
- 将echo命令通过管道传过来的数据作为cat命令的输入，将标准输出作为命令的输出
  ```shell
  $ echo 'hi' | cat
  ```
  
- 将echo命令的输出从默认的标准输出重定向到一个普通文件
  ```shell
  $ echo 'hello world' > redirect
  $ cat redirect
  ```
> 管道默认是连接前一个命令的输出到下一个命令的输入，而重定向通常是需要一个文件来建立两个命令的连接

### 标准错误重定向
标准错误重定向指的是改变原本打印在shell窗口的错误信息的输出位置

```shell
# 使用cat 命令同时读取两个文件，其中一个存在，另一个不存在
$ cat Documents/test.c hello.c

#include <stdio.h>

int main()
{
	print("hello world \n");
	return 0;
}
cat: hello.c: 没有那个文件或目录
```
> 你可以看到除了正确输出了前一个文件的内容，还在末尾出现了一条错误信息

```shell
# 下面我们将输出重定向到一个文件
$ cat Documents/test.c hello.c > somefile

cat: hello.c: 没有那个文件或目录
```
> 这里依然出现了那条错误信息，标准输出和标准错误虽然都指向终端屏幕，实际它们并不一样。那有的时候我们就是要隐藏某些错误或者警告.
  **通过文件描述实现**

```shell
# 将标准错误重定向到标准输出，再将标准输出重定向到文件，注意要将重定向到文件写到前面
$ cat Documents/test.c hello.c >somefile  2>&1
# 或者只用bash提供的特殊的重定向符号"&"将标准错误和标准输出同时重定向到文件
$ cat Documents/test.c hello.c &>somefilehell
```
> 注意你应该在输出重定向文件描述符前加上&,否则shell会当做重定向到一个文件名为1的文件中

### 使用tee命令同时重定向到多个文件
tee 可以将输出同时重定向到文件和打印在终端

```shell
$ echo 'hello world' | tee hello
```

### 永久重定向
使用exec命令实现“永久”重定向。exec命令的作用是使用指定的命令替换当前的 Shell，即使用一个进程替换当前进程，或者指定新的重定向：

```shell
# 先开启一个子 Shell
# 没有打开新的shell窗口，只是一个子进程
$ zsh
# 使用exec替换当前进程的重定向，将标准输出重定向到一个文件
$ exec 1>somefile
# 后面你执行的命令的输出都将被重定向到文件中,直到你退出当前子shell，或取消exec的重定向（后面将告诉你怎么做）
$ ls
$ exit
$ cat somefile

abc
amz
Apps
catkin_ws
.....
```

### 创建输出文件描述符
在 Shell 中有9个文件描述符。上面我们使用了也是它默认提供的0,1,2号文件描述符。另外我们还可以使用3-8的文件描述符，只是它们默认没有打开而已。你可以使用下面命令查看当前 Shell 进程中打开的文件描述符：

```shell
$ cd /dev/fd/;ls -Al
```

- 使用exec命令可以创建新的文件描述符：

  ```shell
  $ zsh
  $ exec 3>somefile
  # 先进入目录，再查看，否则你可能不能得到正确的结果，然后再回到上一次的目录
  $ cd /dev/fd/;ls -Al;cd -
  # 注意下面的命令>与&之间不应该有空格，如果有空格则会出错
  $ echo "this is test" >&3
  $ cat somefile
  $ exit
  
  # ls -Al
  lrwx------ 1 likun likun 64 6月  23 19:07 0 -> /dev/pts/1
  lrwx------ 1 likun likun 64 6月  23 19:07 1 -> /dev/pts/1
  lrwx------ 1 likun likun 64 6月  23 19:07 10 -> /dev/pts/1
  lrwx------ 1 likun likun 64 6月  23 19:07 2 -> /dev/pts/1
  l-wx------ 1 likun likun 64 6月  23 19:07 3 -> /home/likun/somefile

  ```

### 关闭文件描述符
```shell
$ exec 3>&-
$ cd /dev/fd;ls -Al;cd -
```
  
### 完全屏蔽命令的输出
在 Linux 中有一个被称为“黑洞”的设备文件,所有导入它的数据都将被“吞噬”。
> 在类 UNIX 系统中，/dev/null，或称空设备，是一个特殊的设备文件，它通常被用于丢弃不需要的输出流，或作为用于输入流的空文件，这些操作通常由重定向完成。读取它则会立即得到一个EOF。

我们可以利用设个/dev/null屏蔽命令的输出：
```shell
# nefile 和 1 都不存在
$ cat Documents/test.c nefile 1>/dev/null 2>&1
```

### 使用 xargs 分割参数列表 
> xargs 是一条 UNIX 和类 UNIX 操作系统的常用命令。它的作用是将参数列表转换成小块分段传递给其他命令，以避免参数列表过长的问题。

```shell
$ cut -d: -f1 < /etc/passwd | sort | xargs echo
# 重定向符可以去掉
```
> 上面这个命令用于将/etc/passwd文件按:分割取第一个字段排序后，使用echo命令生成一个列表。



