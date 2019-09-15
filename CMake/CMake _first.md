# 使用Cmake构建的本质

现有一个项目包含三个文件：

- main.cpp 
- sum.cpp
- sum.h

sum函数在sum.h头文件内声明，在sum.cpp中定义。

main.cpp 中使用`#include <sum.h> `

sum.cpp有可能被构建为库文件（libsum.so/ libsum.a）

本质：

通过各种操作告诉Cmake这些文件都在哪里，需要生成哪些库文件，然后main.cpp需要和哪些文件或者库文件链接以生成最终的可执行文件 main。

另外还可以命令Cmake将上面提到的文件复制到某些特定的或者自定义的文件夹，即安装

# 目录结构变式

## p1





