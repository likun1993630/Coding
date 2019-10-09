在 ROS 中使用Eigen库

Ubuntu下，Eigen库默认被安装在 `/usr/include/eigen3/`,使用Eigen 库只需要包含头文件，编译时不需要链接。

即，Eigen头文件是hpp，包含实现。

```
/usr/include/eigen3/
.
├── Eigen
│   ├── Cholesky
│   ├── CholmodSupport
│   ├── Core
│   ├── Dense
│   ├── Eigen
│   ├── Eigenvalues
│   ├── Geometry
│   ├── Householder
│   ├── IterativeLinearSolvers
│   ├── Jacobi
│   ├── LU
│   ├── MetisSupport
│   ├── OrderingMethods
│   ├── PardisoSupport
│   ├── PaStiXSupport
│   ├── QR
....
```

需要在*CMakeLists.txt* 中加入内容

```cmake
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
```

之后即可在C++源文件中使用Eigen头文件，比如：

```cpp
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
```

> 参考 《A Systematic Approach to Learning Robot Programming with ROS》 

