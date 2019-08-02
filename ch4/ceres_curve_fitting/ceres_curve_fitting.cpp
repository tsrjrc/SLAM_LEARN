#include <iostream>
#include <chrono>
#include <fstream>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <gnuplot-iostream.h>

using namespace std;

#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>


struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST ( double x,double y ) : _x(x),_y(y){}
  
  template <typename T>
  bool operator()(const T* abc, T* residual) const
  {
    static int i = 0;
    residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
//     cout<<i++<<":"<<abc[0]<<" "<<abc[1]<<" "<<abc[2]<<endl;
    return true;
  }
  const double _x, _y;
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

  
  //构建最小二乘问题
  ceres::Problem problem;
  
  for(int i=0;i<N;i++)
  {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(new CURVE_FITTING_COST(xdata[i],ydata[i])),
      nullptr,
      abc);
  }
  
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  
  ceres::Solve(options,&problem,&summary);
  
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout << "所用时间" << time_used.count() << "s" <<endl;
  
  
//   
   cout << summary.BriefReport() <<endl;
  
  for(auto a:abc) cout<<a<<" ";
  cout<<endl;
  
  std::vector<std::pair<double, double> > mylinepoints1,mylinepoints2;
  for(int i=0;i<N;i++)
  {
    double x = i/100.0;
    
    mylinepoints1.push_back(std::make_pair(x, exp(a*x*x+b*x+c)));
    mylinepoints2.push_back(std::make_pair(x, exp(abc[0]*x*x+abc[1]*x+abc[2])));
  }
  Gnuplot gp;
  gp << "set title '曲线'\n";
  gp << "set xlabel 'X'\nset ylabel 'Y'\n";
  gp << "set xrange [-0.2:1.2]\nset yrange [-10:80]\n";
  gp << "plot" << gp.file1d(mypoints) << "with points pointsize 0.5 pointtype 7 title 'mypoints'," 
      << gp.file1d(mylinepoints1) << "with linespoints pointsize 0.1 linecolor 3 linewidth 0.5 title 'mylinepoints1'," 
      << gp.file1d(mylinepoints2) << "with linespoints pointsize 0.1 linecolor 7 linewidth 0.5 title 'mylinepoints2'" << endl;

  return 0;
}
