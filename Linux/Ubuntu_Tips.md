## 在某个文件夹内寻找所有文件所包含的某个字段

```shell
find . -type f | grep -H "Sending Fusion filter command to begin transmission"
find . -type f |xargs cat > 1.txt << EOF
grep -nr "Sending Fusion filter command to begin transmission" ./
```

