#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc,char **argv)
{
   // 读取argv[1]指定的图像
  Mat image;
  image = imread(argv[1]);
  
  if(image.data == nullptr)
  {
      cerr << "read error." << endl;
      return 0;
  }
  
  // 文件顺利读取, 首先输出一些基本信息
  cout<<"图像宽为"<< image.cols <<",高为"<<image.rows<<" ,通道数为 "<<image.channels()<<endl;
  imshow("image",image);
  waitKey(0);
  
  //判断 image 类型
  if(image.type() != CV_8UC1 && image.type() != CV_8UC3)
  {
    //图像类型不符合；
    cout << "请输入一张彩色或者灰度图。" << endl;
    return 0; 
  }
  
  //遍历图像，计算时间
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for(size_t y=0;y<image.rows;y++)
  {
    for(size_t x=0;x<image.cols;x++)
    {
      unsigned char* row_ptr = image.ptr<unsigned char>(y);
      unsigned char* data_ptr = &row_ptr[x*image.channels()];
      
      for(int i=0;i!=image.channels();i++)
      {
	unsigned char data = data_ptr[i];
      }
      
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout << "遍历视觉" << time_used.count() << "s" <<endl;
  waitKey(0);
  
  //mat 拷贝 直接赋值不拷贝
  Mat image_another = image;
  image_another(Rect(0,0,400,400)).setTo(0);
  imshow("image",image);
  waitKey(0);
  
  Mat image_clone = image.clone();
  image_clone(Rect(0,0,400,400)).setTo(255);  
  imshow("image",image);
  imshow("image_clone",image_clone);
  waitKey(0);
  
  destroyAllWindows();
  return 0;
  
}