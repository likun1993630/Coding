# main函数的argc和argv参数

```cpp
int main(int argc, char** argv)
```

argc 和 argv用来接受执行main函数时给main函数传递的参数。

比如在命令行运行可执行文件，在可执行文件后面写的参数就被传递给argv和argc。

argc 是参数的数目

argv 是char指针数组，该数组的元素是指针，该指针指向各个参数

argv的第一项argv[0]永远是该可执行文件的所在目录,argv[1]开始才是参数。

```cpp
#include <iostream>
#include <string>

using std::string;
using std::cout;
using std::endl;

int main(int argc, char** argv)//实参列表
{
    string str;
    for (int i = 0; i != argc; ++i) {
        str += argv[i];
        str += " ";
    }
 	
 	cout << argc << endl;
    cout << str << endl;
    return 0;
}
```

```shell
# 执行：
$ ./a.out para1 para2

# 结果：
3
./a.out para1 para2 
```

