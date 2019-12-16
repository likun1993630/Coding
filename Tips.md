sublime： 
- ctrl+shift+L 选择多行，然后方向键移动光标至首行
- ctrl+f 切换成 * 号，即正则表达式，输入 > ,点击全选，光标处于每行的行首

github文件名空格引用：
- 使用 `&#32;` 代替空格

linux搜索文件内的指定字符：

```sh
$ find . -type f | grep -H "Sending Fusion filter command to begin transmission"

$ find . -type f |xargs cat > 1.txt << EOF

$ grep -nr "Sending Fusion filter command to begin transmission" ./
```

