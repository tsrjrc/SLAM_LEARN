#include <iostream>
#include <chrono>
#include <fstream>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream.h>

#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

using namespace std;
#include <opencv2/core/core.hpp>



class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl()
  {
    _estimate << 0,0,0;
  }
  
  virtual void oplusImpl(const double* update)
  {
    static int i= 0;
    _estimate+=Eigen::Vector3d(update);
//     cout << i++ << ": " << Eigen::Vector3d(update).transpose()<<endl;
  }
  virtual bool read(istream& in){}
  virtual bool write(ostream& out)const{}
  
};

class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x):BaseUnaryEdge(),_x(x){};
  void computeError()
  {
    const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0,0) = _measurement - std::exp(abc(0,0)*_x*_x+abc(1,0)*_x+abc(2,0));
  }
  virtual bool read(istream& in){}
  virtual bool write(ostream& out)const{}
public:
  double _x;
  
};

int main(int argc ,char** argv)
{
  
  double a=1.0,b=2.0,c=1.0;
  int N = 100;
  double w_sigma = 1.0;
  cv::RNG rng;
  double abc[3]={0};

  vector<double> xdata,ydata;
  cout << "产生数据" <<endl;
  
  
  std::vector<std::pair<double, double> > mypoints;
  for(int i=0;i<N;i++)
  {
    double x = i/100.0;
    xdata.push_back(x);
    ydata.push_back(exp(a*x*x+b*x+c)+rng.gaussian(w_sigma));
    
    mypoints.push_back(std::make_pair(xdata[i], ydata[i]));
    
  }

  //构建图优化，先设定g2o
  //矩阵块
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
  //配置求解器
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);
  
//   //梯度下降
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm( solver );
  optimizer.setVerbose( true );
  
  CurveFittingVertex* v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0,0,0));
  v->setId(0);
  optimizer.addVertex(v);
  
  for(int i=0;i<N;i++)
  {
    CurveFittingEdge* edge = new CurveFittingEdge( xdata[i] );
    edge->setId(i);
    edge->setVertex(0,v);
    edge->setMeasurement(ydata[i]);
    
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma));
    optimizer.addEdge(edge);
  }
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout << "所用时间" << time_used.count() << "s" <<endl;
  
  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "abc_estimate:" << abc_estimate.transpose() <<endl;
 
  std::vector<std::pair<double, double> > mylinepoints1,mylinepoints2;
  for(int i=0;i<N;i++)
  {
    double x = i/100.0;
    
    mylinepoints1.push_back(std::make_pair(x, exp(a*x*x+b*x+c)));
    mylinepoints2.push_back(std::make_pair(x, exp(abc_estimate(0,0)*x*x+abc_estimate(1,0)*x+abc_estimate(2,0))));
  }
  
  Gnuplot gp;
  gp << "set title '曲线'\n";
  gp << "set xlabel 'X'\nset ylabel 'Y'\n";
  gp << "set xrange [-0.2:1.2]\nset yrange [-10:80]\n";
  gp << "plot" << gp.file1d(mypoints) << "with points pointsize 0.5 pointtype 7 title 'mypoints'," 
     << gp.file1d(mylinepoints1) << "with linespoints pointsize 0.1 linecolor 3 linewidth 0.5 title 'mylinepoints1'," 
     << gp.file1d(mylinepoints2) << "with linespoints pointsize 0.1 linecolor 7 linewidth 0.5 title 'mylinepoints2'" 
     << endl;

  return 0;
}
