//
// Created by 高翔 on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace std;

string image_file = "../test.png";   // 请确保路径正确

int main(int argc, char **argv) {

    // 本程序需要你自己实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file,0);   // 图像是灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
    cv::Mat image_undistort1 = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
    
    
    
    cv::Mat matrix;
    cv::Mat coeff;
    cv::Mat new_matrix;
    matrix = cv::Mat(3, 3, CV_64F);
    matrix.at<double>(0, 0) = fx;
    matrix.at<double>(0, 1) = 0;
    matrix.at<double>(0, 2) = cx;
    matrix.at<double>(1, 0) = 0;
    matrix.at<double>(1, 1) = fy;
    matrix.at<double>(1, 2) = cy;
    matrix.at<double>(2, 0) = 0;
    matrix.at<double>(2, 1) = 0;
    matrix.at<double>(2, 2) = 1;
    coeff = cv::Mat(1, 4, CV_64F);
    coeff.at<double>(0, 0) = k1;
    coeff.at<double>(0, 1) = k2;
    coeff.at<double>(0, 2) = p1;
    coeff.at<double>(0, 3) = p2;

    
   
    cv::undistort(image,image_undistort1,matrix, coeff);
    cv::imshow("image_undistort1", image_undistort1);
    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            double u_distorted = 0, v_distorted = 0;
            // TODO 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted) (~6 lines)
            // start your code here
	    //计算归一化后的xy 及r
	    double x = (u-cx)/fx;
	    double y = (v-cy)/fy;
            double r = sqrt(pow(x,2)+pow(y,2));
// 	    cout << "x:" << x << " y:" << y << " r:" << r << endl;
	    double xd = x*(1+k1*pow(r,2)+k2*pow(r,4))+2*p1*x*y+p2*(pow(r,2)+2*pow(x,2));
	    double yd = y*(1+k1*pow(r,2)+k2*pow(r,4))+p1*(pow(r,2)+2*pow(y,2))+2*p2*x*y;
	    
	    u_distorted = fx*xd + cx;
	    v_distorted = fy*yd + cy;
	    
//  	    cout << " xd:" << xd << " yd:" << yd << " u_distorted:" << u_distorted << " v_distorted:" << v_distorted << endl;
// 	    cv::waitKey();
            // end your code here

            // 赋值 (最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    // 画图去畸变后图像
    cv::imshow("image", image);
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();

    return 0;
}
