#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
 
using namespace std;
 
 
int main(int argc, char **argv) {
  
  //导入小萝卜一号数据  
  Eigen::Quaterniond q1 (0.35, 0.2, 0.3, 0.1);
  Eigen::Vector3d t1 (0.3, 0.1, 0.1);
  //导入小萝卜二号数据
  Eigen::Quaterniond q2 (-0.5, 0.4, -0.1, 0.2);
  Eigen::Vector3d t2 (-0.1, 0.5, 0.3);
  //导入小萝卜一号观测观测点数据
  Eigen::Vector3d p (0.5, 0, 0.2);
  //定义小萝卜二号观测观测点坐标
  Eigen::Vector3d o;
  
  //初始化欧式变换矩阵
  Eigen::Isometry3d T_1cw = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_2cw = Eigen::Isometry3d::Identity();
  
  //开始求解
  //归一化四元数
  q1.normalize();
  q2.normalize();
  //输出归一化后信息
  cout<<"q1 = "<< endl << q1.x()<<endl<<q1.y()<< endl <<q1.z()<< endl<<q1.w()<<endl;
  cout<<"q2 = "<< endl << q2.x()<<endl<<q2.y()<< endl <<q2.z()<< endl<<q2.w()<<endl;
  
  //将四元数和位移融合
  T_1cw.rotate(q1);
  T_1cw.pretranslate(t1);
  
  T_2cw.rotate(q2);
  T_2cw.pretranslate(t2);
  
  //输出位姿矩阵信息
  cout << "T_1cw: " << endl << T_1cw.matrix() << endl;
  cout << "T_2cw: " << endl << T_2cw.matrix() << endl;
  
  //求解o坐标
  o = T_2cw * T_1cw.inverse() * p;
  
  //输出o的坐标
  cout << "o = " << o.transpose() << endl;
  
    return 0;
}
