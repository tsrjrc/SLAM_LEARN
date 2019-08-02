#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include "unistd.h"

using namespace std;

// path to trajectory file
string trajectory_file = "../trajectory.txt";
string groundtruth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses3;
    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream trajectoryfile(trajectory_file);
    if(!trajectoryfile) return 0;
    ifstream groundtruthfile(groundtruth_file);
    if(!groundtruthfile) return 0;
    ifstream estimatedfile(estimated_file);
    if(!estimatedfile) return 0;
    
    for(int i=0;i<620;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {
      double data[8];
      for(int j=0;j<8;j++)
      {
	trajectoryfile>>data[j];
	Eigen::Quaterniond q = Eigen::Quaterniond(data[7],data[4],data[5],data[6]);
	Eigen::Vector3d t(data[1],data[2],data[3]);
	Sophus::SE3 seqt(q,t);
	poses1.push_back(seqt);
      }
    }
    double error = 0;
    for(int i=0;i<620;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {
      double data1[8]={0}; double data2[8]={0};
      for(int j=0;j<8;j++)
      {
	groundtruthfile>>data1[j];
	estimatedfile>>data2[j];
      }
      Eigen::Quaterniond q1 = Eigen::Quaterniond(data1[7],data1[4],data1[5],data1[6]);
      Eigen::Vector3d t1(data1[1],data1[2],data1[3]);
      Sophus::SE3 seqt1(q1,t1);
      poses2.push_back(seqt1);

      Eigen::Quaterniond q2 = Eigen::Quaterniond(data2[7],data2[4],data2[5],data2[6]);
      Eigen::Vector3d t2(data2[1],data2[2],data2[3]);
      Sophus::SE3 seqt2(q2,t2);
      poses3.push_back(seqt2);
      
      
      Eigen::Matrix<double,4,4> E = (seqt1.matrix().inverse())*seqt2.matrix();
      Eigen::Matrix<double,3,3> R = E.topLeftCorner<3,3>();
      Eigen::Matrix<double,3,1> t = E.topRightCorner<3,1>();
      Eigen::Quaterniond q(R);
      Eigen::Vector3d tt(t);
      Sophus::SE3 E_Se3(q,tt);
      Sophus::Vector6d E_Log = E_Se3.log(); 
      
      double e = E_Log.norm();
      
      error += e*e;
    }
   cout << "error :" << sqrt(error/612) << endl;
   // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses2,poses3);

    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    if (poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1 - (float) i / poses1.size(), 1.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
	
        for (size_t i = 0; i < poses2.size() - 1; i++) {
            glColor3f(1.0f, 0, 0);
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    
}