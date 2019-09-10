# Linux背景知识

1. 在终端shell下输入 cd chmod rosrun等命令时，过程是怎样的：

- `cd`,`chmod`,`rosrun`等命令其实就是可执行文件，并且被放在了某个目录。

- 首先，shell会在环境变量PATH中所包含的目录中按照一定顺序依次在各个目录中查找可执行文件

  ```shell
  ➜ ~ echo $PATH
  /opt/ros/kinetic/bin:/home/likun/bin:/home/likun/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
  # 其中，路径之间以冒号分隔
  ```

- 如果在某一个路径找到名字正确的执行文件则运行，如果所有目录都找不到则报错

2. 使用命令查看可执行命令位置：

   which: 查看命令的所在目录

    ```shell
    ➜ ~ which cd
    cd: shell built-in command
    ➜ ~ which chmod
    /bin/chmod
    ➜ ~ which catkin_create_pkg 
    /usr/bin/catkin_create_pkg
    ➜ ~ which rosrun
    /opt/ros/kinetic/bin/rosrun
    ```
   
   whereis: 查看命令的所在目录,与这条命令相关联的代码以及帮助文档放在哪
   
   ```shell
   ➜ ~ whereis tree
   tree: /usr/bin/tree /usr/share/man/man1/tree.1.gz
   ➜ ~ whereis catkin_create_pkg 
   catkin_create_pkg: /usr/bin/catkin_create_pkg
   ➜ ~ whereis chmod            
   chmod: /bin/chmod /usr/share/man/man1/chmod.1.gz /usr/share/man/man2/chmod.2.gz
   ```


3. shell 命令一般放在哪个目录：

   ```
   /bin、/usr/bin : 这个目录下存放的命令是针对普通的用户都可以使用命令。
   
   /sbin、/usr/sbin：这个目录下存放的针对root用户才能使用的命令。
   ```

4. usr目录：

   ```
   /usr/    包括与系统用户直接有关的文件和目录   
       /usr/bin/        基于用户命令的可执行文件(应用程序)
       /usr/sbin/        管理员应用程序
       /usr/include        编译应用程序所需要的头文件
       /usr/lib/        应用程序库文件（常用的动态链接库和软件包的配置文件）
       /usr/share/        应用程序资源文件
       /usr/src/        应用程序源代码
       /usr/doc        存放文档的目录
       /usr/man        存放帮助文档的目录
       /usr/local/soft/        用户程序
       /usr/local/bin        本地增加的命令
       /usr/local/lib        本地增加的库根文件系统
       /usr/X11R6        图形界面系统(存放x windows的目录)
   ```

