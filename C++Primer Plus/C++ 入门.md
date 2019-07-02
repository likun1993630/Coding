## 第一个c++ 程序

```cpp
// 第一个c++程序
#include <iostream>

int main()
{
	using namespace std;
	cout << "hello world.";
	cout << endl;

	return 0;
}
```

使用g++ 编译：
```shell
# cd 到文件目录
$ g++ main.cpp
```
运行：
```shell
$ a.exe
# 或者
$ a
```
> 无参数传递时，c++ 习惯使用 int main（），而不用 int main(void)，这样写并没有错误，只是不符合习惯。

> `""` 双引号为字符串

> 
