#include <iostream>



//Eigen 代数库
#include <Eigen/Core>
#include <Eigen/Dense>



using namespace std;



int main ( int argc, char** argv )
{

  //声明一个float 2*3的矩阵 2行3列 2row - 3col Matrix
  Eigen::Matrix<float, 2, 3> matrix_23;
  Eigen::Vector3f vector_3f;
  Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
  
  
  
  matrix_23 << 1,2,3,4,5,6;
  vector_3f << 1,2,3;
  matrix_dynamic << 1,2,3,4,5,6;
  cout << "matrix_23:" << matrix_23 << endl;
  cout << "vector_3f:" << vector_3f << endl;
  cout << "matrix_dynamic:" << matrix_dynamic << endl;
  
  
  return 0;
}

