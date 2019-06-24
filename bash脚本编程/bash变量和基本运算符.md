## 变量定义
### 概念

变量的名字就是变量保存值的地方。引用变量的值就叫做变量替换。

如果 variable 是一个变量的名字，那么 $variable 就是引用这个变量的值，即这变量所包含的数据。

variable事实上只是variable 事实上只是 variable事实上只是{variable} 的简写形式。在某些上下文中 variable可能会引起错误，这时候你就需要用variable 可能会引起错误，这时候你就需要用 variable可能会引起错误，这时候你就需要用{variable} 了。

### 定义变量

定义变量时，变量名不加美元符号（$，PHP语言中变量需要），如： myname="shiyanlou"

注意:

变量名和等号之间不能有空格。同时，变量名的命名须遵循如下规则：

- 首个字符必须为字母（a-z，A-Z）。
- 中间不能有空格，可以使用下划线（_）。
- 不能使用标点符号。
- 不能使用bash里的关键字（可用help命令查看保留关键字）。

除了直接赋值，还可以用语句给变量赋值，如：
```
for file in `ls /etc` 
```

## 使用变量 
```shell
myname="shiyanlou"
echo $myname
echo ${myname}
echo ${myname}Good
echo $mynameGood

myname="miao"
echo ${myname}
```

> 加花括号帮助解释器识别变量的边界，若不加，解释器会把mynameGood当成一个变量（值为空）

> 推荐给所有变量加花括号

> 已定义的变量可以重新被定义

## 只读变量 

使用 `readonly` 命令可以将变量定义为只读变量，只读变量的值不能被改变。 下面的例子尝试更改只读变量，结果报错：

```shell
#!/bin/bash
myUrl="http://www.shiyanlou.com"
readonly myUrl
myUrl="http://www.shiyanlou.com"
```
结果如下：
```shell
/bin/sh: NAME: This variable is read only.
```

## 特殊变量
- 局部变量
  这种变量只有在代码块或者函数中才可见。
- 环境变量
  这种变量将影响用户接口和 shell 的行为。
  在通常情况下，每个进程都有自己的“环境”，这个环境是由一组变量组成的，这些变量中存有进程可能需要引用的信息。在这种情况下，shell 与一个一般的进程没什么区别。
- 位置参数
  从命令行传递到脚本的参数：0，1，2，3...
  0就是脚本文件自身的名字，1 是第一个参数，2是第二个参数，3 是第三个参数，然后是第四个。9之后的位置参数就必须用大括号括起来了，比如，{10}，{11}，{12}。
  
  ```
  $# ： 传递到脚本的参数个数
  $* ： 以一个单字符串显示所有向脚本传递的参数。与位置变量不同,此选项参数可超过 9个
  $$ : 脚本运行的当前进程 ID号
  $! ： 后台运行的最后一个进程的进程 ID号
  $@ ：与$*相同,但是使用时加引号,并在引号中返回每个参数
  $- ： 显示shell使用的当前选项,与 set命令功能相同
  $? ： 显示最后命令的退出状态。 0表示没有错误,其他任何值表明有错误。
  ```
### 位置参数实例
在执行 Shell 脚本时，向脚本传递参数，脚本内获取参数的格式为：$n。n 代表一个数字，1 为执行脚本的第一个参数，2 为执行脚本的第二个参数，以此类推……
```shell
$ vim test30.sh
```
输入代码（中文皆为注释，不用输入）：
```shell
#!/bin/bash

# 作为用例, 调用这个脚本至少需要10个参数, 比如:
# bash test.sh 1 2 3 4 5 6 7 8 9 10
MINPARAMS=10
echo
echo "The name of this script is \"$0\"."
echo "The name of this script is \"`basename $0`\"."
echo

if [ -n "$1" ]              # 测试变量被引用.
then
echo "Parameter #1 is $1"  # 需要引用才能够转义"#"
fi 

if [ -n "$2" ]
then
echo "Parameter #2 is $2"
fi 

if [ -n "${10}" ]  # 大于$9的参数必须用{}括起来.
then
echo "Parameter #10 is ${10}"
fi 

echo "-----------------------------------"
echo "All the command-line parameters are: "$*""

if [ $# -lt "$MINPARAMS" ]
then
 echo
 echo "This script needs at least $MINPARAMS command-line arguments!"
fi  

echo

exit 0
```

> \`basename $0\` 的含义是只返回去掉路径的文件名
  比如当，$0 返回值为/home/likun/test.sh，则basename $0 返回 test.sh

运行代码：
```shell
$ bash test30.sh 1 2 10


The name of this script is "test.sh".
The name of this script is "test.sh".

Parameter #1 is 1
Parameter #2 is 2
-----------------------------------
All the command-line parameters are: 1 2 10

This script needs at least 10 command-line arguments!
```

# 基本运算符
##算数运算符

假定变量 a 为 10，变量 b 为 20：

| 运算符 | 说明                                          | 举例                          |
|--------|-----------------------------------------------|-------------------------------|
| +      | 加法                                          | `expr $a + $b` 结果为 30。    |
| -      | 减法                                          | `expr $a - $b` 结果为 -10。   |
| *      | 乘法                                          | `expr $a \* $b` 结果为  200。 |
| /      | 除法                                          | `expr $b / $a` 结果为 2。     |
| %      | 取余                                          | `expr $b % $a` 结果为 0。     |
| =      | 赋值                                          | a=$b 将把变量 b 的值赋给 a。  |
| ==     | 相等。用于比较两个数字，相同则返回 true。     | [ $a == $b ] 返回 false。     |
| !=     | 不相等。用于比较两个数字，不相同则返回 true。 | [ $a != $b ] 返回 true。      |

```shell
$vim test.sh
```
输入代码：
```shell
#!/bin/bash

a=10
b=20
val=`expr $a + $b`
echo "a + b : $val"
val=`expr $a - $b`
echo "a - b : $val"
val=`expr $a \* $b`
echo "a * b : $val"
val=`expr $b / $a`
echo "b / a : $val"
val=`expr $b % $a`
echo "b % a : $val"
if [ $a == $b ]
then
   echo "a == b"
fi
if [ $a != $b ]
then
   echo "a != b"
fi
```
运行
```shell
$bash test.sh

a + b : 30
a - b : -10
a * b : 200
b / a : 2
b % a : 0
a != b
```
> - 原生bash不支持简单的数学运算，但是可以通过其他命令来实现，例如 awk 和 expr，expr 最常用。
> - expr 是一款表达式计算工具，使用它能完成表达式的求值操作。
> - 注意使用的反引号（esc键下边）
> - 表达式和运算符之间要有空格$a + $b写成$a+$b不行
> - 条件表达式要放在方括号之间，并且要有空格[ $a == $b ]写成[$a==$b]不行
> - 乘号（\*）前边必须加反斜杠（\)才能实现乘法运算

