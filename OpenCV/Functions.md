## namedWindow

**功能说明：**namedWindow（）的功能就是新建一个显示窗口。可以指定窗口的类型。

**API详解：**原型：`void nameWindow(const string& winname,int flags = WINDOW_AUTOSIZE) ;`

```cpp
#include <opencv2/opencv.hpp>
using namespace cv;
int main()
{
    Mat img;
    img = imread("Marker2.jpg",1);//参数1：图片路径。参数2:显示原图
    namedWindow("窗口1",CV_WINDOW_NORMAL);
    /*注释
    参数1：窗口的名字
    参数2：窗口类型，CV_WINDOW_AUTOSIZE 时表明窗口大小等于图片大小。不可以被拖动改变大小。
    CV_WINDOW_NORMAL 时，表明窗口可以被随意拖动改变大小。
    */

    imshow("窗口1",img);//在“窗口1”这个窗口输出图片。
    waitKey(50000);//等待50秒，程序自动退出。改为0，不自动退出。
    return 0;
}
```

