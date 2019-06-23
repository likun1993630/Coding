## 内建命令与外部命令

- 内建命令
  内建命令实际上是 shell 程序的一部分，其中包含的是一些比较简单的 Linux 系统命令，这些命令是写在bash源码的builtins里面的，由 shell 程序识别并在 shell 程序内部完成运行，通常在 Linux 系统加载运行时 shell 就被加载并驻留在系统内存中。而且解析内部命令 shell 不需要创建子进程，因此其执行速度比外部命令快。比如：history、cd、exit 等等。

- 外部命令
  外部命令是 Linux 系统中的实用程序部分，因为实用程序的功能通常都比较强大，所以其包含的程序量也会很大，在系统加载时并不随系统一起被加载到内存中，而是在需要时才将其调入内存。虽然其不包含在 shell 中，但是其命令执行过程是由 shell 程序控制的。外部命令是在 Bash 之外额外安装的，通常放在/bin，/usr/bin，/sbin，/usr/sbin等等。比如：ls、vi等。
  
  > 简单来说就是：一个是天生自带的天赋技能，一个是后天得来的附加技能。我们可以使用　type 命令来区分命令是内建的还是外部的。例如这两个得出的结果是不同的
  
  ```shell
  $ type exit
  $ type vim
  $ type ls
  
  exit is a shell builtin
  vim is /usr/bin/vim
  ls is an alias for ls --color=tty
  ```
  ```shell
  #得到这样的结果说明是内建命令，正如上文所说内建命令都是在 bash 源码中的 builtins 的.def中
  xxx is a shell builtin
  #得到这样的结果说明是外部命令，正如上文所说，外部命令在/usr/bin or /usr/sbin等等中
  xxx is /usr/bin/xxx
  #若是得到alias的结果，说明该指令为命令别名所设定的名称；
  xxx is an alias for xx --xxx
  ```
  
## 帮助命令的使用 

- help 命令
  >  zsh 中内置没有 help 命令，可以进入 bash 中，在 bash 中内置有该命令
  
  ```shell
  $ bash
  $ help ls
  
  bash: help: 没有与 `ls' 匹配的帮助主题。尝试 `help help' 或 `man -k ls' 或 `info ls'。
  ```
  > 因为 help 命令是用于显示 shell 内建命令的简要帮助信息。帮助信息中显示有该命令的简要说明以及一些参数的使用以及说明，一定记住 help 命令只能用于显示内建命令的帮助信息。
  
  > 对于外部命令，基本上都有一个参数--help,这样就可以得到相应的帮助。
  
  ```shell
  $ ls --help
  ```
  
- man 命令 
  ```shell
  $ man ls
  
  LS(1)          User Commands       LS(1)
  NAME
       ls - list directory contents
  .....
  
  ```
  > man 得到的内容比用 help 更多更详细，而且　man　没有内建与外部命令的区分，因为 man 工具是显示系统手册页中的内容，也就是一本电子版的字典，这些内容大多数都是对命令的解释信息，还有一些相关的描述。通过查看系统文档中的 man 也可以得到程序的更多相关信息和 Linux 的更多特性。
  
  > 最左上角显示“ LS （1）”，在这里，“ LS ”表示手册名称，而“（1）”表示该手册位于第一章节。其他章节内容参见下表：
  
| 章节数 | 说明                                               |
|--------|----------------------------------------------------|
| 1      | Standard commands （标准命令）                     |
| 2      | System calls （系统调用）                          |
| 3      | Library functions （库函数）                       |
| 4      | Special devices （设备说明）                       |
| 5      | File formats （文件格式）                          |
| 6      | Games and toys （游戏和娱乐）                      |
| 7      | Miscellaneous （杂项）                             |
| 8      | Administrative Commands （管理员命令）             |
| 9      | 其他（Linux特定的）， 用来存放内核例行程序的文档。 |


  
