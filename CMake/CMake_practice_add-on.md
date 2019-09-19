2. 需要设置环境变量，以执行hello和runhello.sh

   ```shell
   ➜ ~ export PATH=$PATH:/tmp/t2/usr/bin
   ➜ ~ hello
   Hello World form t1 Main! 
   ➜ ~ runhello.sh 
   Hello World form t1 Main! 
   ```

3. 什么是标准路径

   

3. linux下C/C++编译时系统搜索 include 和 链接库 文件路径的指定

   1. include<头文件名>和include"头文件名"

      如：include<stdio.h>和include"stdio.h"

      前者（使用<>），来引用stdio.h文件，是首先检索标准路径，看看这些文件夹下是否有该头文件；如果没有，也不会检索当前文件所在路径，并将报错。

      后者（使用""），来引用stdio.h文件，是首先检索文件的当前路径；如果没有，再检索标准路径，看看这些文件夹下是否有该头文件。

   2. linux下，上述标准路径有：/usr/include，/usr/local/include。

   3. <sys/头文件名>。如<sys/io.h>，<net/ethernet.h>等。其中，前面的字符串（如sys,net）表示标准路径下的文件夹名，后面的字符串（如io.h,ethernet.h），表示在linux标准路径下的各文件夹下的头文件名，如sys文件夹下的io.h文件，即我们可以在/usr/include/sys目录下发现io.h文件。

      如下图所示：/usr/include/ 文件内既有文件夹内存放的头文件也有直接在include目录下的头文件，在c++源文件中使用include命令时，要遵循目录结构。

      ```
      net
      ├── ethernet.h
      ├── if_arp.h
      ├── if.h
      ├── if_packet.h
      ├── if_ppp.h
      ├── if_shaper.h
      ├── if_slip.h
      ├── ppp-comp.h
      ├── ppp_defs.h
      └── route.h
      
      此时如果需要ethernet头文件，应该使用如下语句：
      #include <net/ethernet.h>
      ```

      ![1568152634860](res/1568152634860.png)




