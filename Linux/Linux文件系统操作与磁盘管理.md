> 此为入门基础，没有涉及太多理论内容，如有需要应另外补充知识。

## 查看磁盘和目录的容量

- 使用 df 命令查看磁盘的容量
  ```shell
  $ df
  
  文件系统           1K-块     已用      可用 已用% 挂载点
  udev             8153140        0   8153140    0% /dev
  tmpfs            1643980    18284   1625696    2% /run
  /dev/sdb3       30831676 11222420  18020012   39% /
  tmpfs            8219896    53580   8166316    1% /dev/shm
  tmpfs               5120        4      5116    1% /run/lock
  tmpfs            8219896        0   8219896    0% /sys/fs/cgroup
  /dev/nvme0n1p2     97280    32801     64479   34% /boot/efi
  /dev/sdb4       74953600 29900524  41222500   43% /home
  tmpfs            1643980       16   1643964    1% /run/user/122
  tmpfs            1643980      184   1643796    1% /run/user/1000
  /dev/nvme0n1p6 283585532 24921716 258663816    9% /media/likun/SSD2
  ```
  
  > 物理主机上的 /dev/sdb3 是对应着主机硬盘的分区，后面的数字3表示分区号，数字前面的字母 b 表示第几块硬盘（也可能是可移动磁盘），你如果主机上有多块硬盘则可能还会出现 /dev/sdc，/dev/sdd 这些磁盘设备都会在 /dev 目录下以文件的存在形式。
  
  > "1k-块"表示以磁盘块大小的方式显示容量，后面为相应的以块大小表示的已用和可用容量。
  
- 另一种方式
  ```shell
  $ df -h
  
  文件系统        容量  已用  可用 已用% 挂载点
  udev            7.8G     0  7.8G    0% /dev
  tmpfs           1.6G   18M  1.6G    2% /run
  /dev/sdb3        30G   11G   18G   39% /
  tmpfs           7.9G   53M  7.8G    1% /dev/shm
  tmpfs           5.0M  4.0K  5.0M    1% /run/lock
  tmpfs           7.9G     0  7.9G    0% /sys/fs/cgroup
  /dev/nvme0n1p2   95M   33M   63M   34% /boot/efi
  /dev/sdb4        72G   29G   40G   43% /home
  tmpfs           1.6G   16K  1.6G    1% /run/user/122
  tmpfs           1.6G  184K  1.6G    1% /run/user/1000
  /dev/nvme0n1p6  271G   24G  247G    9% /media/likun/SSD2
  ```
  > 可以看到硬盘b（sdb4）的第4分区被挂载到了/home目录下
  
- 使用 du 命令查看目录的容量
  
  ```shell
  # 默认同样以 块 的大小展示
  $ du 
  # 加上`-h`参数，以更易读的方式展示
  $ du -h
  ```
  -d 参数指定查看目录的深度
  
  ```shell
  # 只查看1级目录的信息
  $ du -h -d 0 ~
  # 查看2级
  $ du -h -d 1 ~
  ```
  常用参数
  
  ```shell
  # 来自: http://man.linuxde.net/du
  du -h #同--human-readable 以K，M，G为单位，提高信息的可读性。
  du -a #同--all 显示目录中所有文件的大小。
  du -s #同--summarize 仅显示总计，只列出最后加总的值。
  ```

## 简单的磁盘管理

待学习

  
