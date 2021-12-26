//this file is for testing small functions
#include <iostream>
// #include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include <ceres/ceres.h>
#include <chrono>

#include "ASEKF.h"
#include "RandomVector.h"
#include "KeyFrame.h"

using namespace std;

int main(){
    ASEKF asekf(motion(Eigen::Vector3d(1,1,2),Eigen::Matrix3d:Zero(3)));
    
    
}

// int main(){

//     double x;
//     Eigen::Vector3d a(1,2,2);
//     Eigen::Vector3d b(2,2,1);
//     x=double(a.transpose()*b);
//     cout<<x<<endl;
//     return 0;
// }

// typedef struct RandomVector{
//     RandomVector(){}
//     RandomVector(Eigen::VectorXd hat_, Eigen::MatrixXd P_):hat(hat_),P(P_){}

//     RandomVector compound(RandomVector b){
//     /*
//     输入：两个首尾相接的关系随机向量
//     输出：两个关系随机向量的复合
//     */
//         Eigen::VectorXd ij = this->hat;
//         Eigen::VectorXd jk = b.hat;
//         this->hat = Eigen::Vector3d(jk(0)*cos(ij(2))-jk(1)*sin(ij(2))+ij(0),
//                             jk(0)*sin(ij(2))+jk(1)*cos(ij(2))+ij(1),
//                             ij(2)+jk(2));
//         Eigen::VectorXd ik = this->hat;
//         Eigen::MatrixXd J1,J2;
//         J1 << 1,0,-(ik(1)-ij(1)),
//                 0,1,(ik(0)-ij(0)),
//                 0,0,1;
//         J2 << cos(ij(2)), -sin(ij(2)),0,
//                 sin(ij(2)), cos(ij(2)),0,
//                 0,0,1;
//         this->P = J1*this->P*J1.transpose()+J2*b.P*J2.transpose();
//         return *this;
//     }
//     RandomVector rinverse(){
//     /*
//     输入：关系随机向量
//     输出：逆关系随机向量
//     */
//         Eigen::VectorXd x = this->hat;
//         this->hat << -x(0)*cos(x(2))-x(1)*sin(x(2)),x(0)*sin(x(2))-x(1)*cos(x(2)),-x(2);
//         Eigen::MatrixXd J;
//         J<< -cos(x(2)),-sin(x(2)),x(1),
//             sin(x(2)),-cos(x(2)),-x(0),
//             0,0,-1;
//         this-> P = J*this->P*J.transpose();
//         return *this;
//     }
//     RandomVector tail2tail(RandomVector b){
//         return (this->rinverse()).compound(b);
//     }

//     bool operator ==(const RandomVector& b){
//         return (this->hat).isApprox(b.hat, 1e-5) && (this->P).isApprox(b.P);
//     }

//     Eigen::VectorXd hat;
//     Eigen::MatrixXd P;
// }point, pose, motion, error;

// //定义仿函数　CostFunctor
// struct CURVE_FITTING_COST
// {
//     CURVE_FITTING_COST ( point x, point y ) : _x ( x ), _y ( y ) {}
//     template <typename T> 
//     bool operator() (
//         const T* const abc,     // 模型参数，有3维
//         T* residual ) const     // 残差
//     {
//         // residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
//         residual[0]=T(_x.hat.transpose()*_y.hat);
//         return true;
//     }
//     const point _x,_y;
//     // const double _x, _y;    // x,y数据
// };

// int main ( int argc, char** argv )
// {
//     double a=1.0, b=2.0, c=1.0;         // 真实参数值
//     int N=100;                          // 数据点
//     double w_sigma=1.0;                 // 噪声Sigma值
//     cv::RNG rng;                        // OpenCV随机数产生器
//     double abc[3] = {0,0,0};            // abc参数的估计值

//     // vector<double> x_data, y_data;      // 数据
//     vector<point> x_data, y_data;      // 数据

//     cout<<"generating data: "<<endl;
//     for ( int i=0; i<N; i++ )
//     {
//         double x = i/100.0;
//         point tmp(Eigen::Vector3d(1,3,3),Eigen::Matrix3d::Zero(3,3));
//         tmp.hat(1)+=rng.gaussian ( w_sigma );
//         x_data.push_back ( tmp );
//         tmp.hat(2)+=rng.gaussian ( w_sigma );
//         y_data.push_back ( tmp );
//         // cout<<x_data[i]<<" "<<y_data[i]<<endl;
//     }

//     // 构建最小二乘问题
//     ceres::Problem problem;
//     for ( int i=0; i<N; i++ )
//     {
//         //这里注意与前面的介绍对应着看
//         problem.AddResidualBlock (
//             // *cost_function 使用自动求导，模板参数：偏差类型，输出维度，输入维度，维数要与前面struct中一致
//             new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> ( 
//                 new CURVE_FITTING_COST ( x_data[i], y_data[i] )
//             ),
//             nullptr,  // *loss_function，为空。或者设置为huberloss，new ceres::HuberLoss(huber_scale)
//             abc       // 待估计参数
//         );
//     }

//     // 配置求解器
//     ceres::Solver::Options options;     // 这里有不少配置项能够填
//     options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
//     options.minimizer_progress_to_stdout = true;   // 输出到cout

//     ceres::Solver::Summary summary;                // 优化信息
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//     ceres::Solve ( options, &problem, &summary );  // 开始优化
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
//     cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

//     // 输出结果
//     cout<<summary.BriefReport() <<endl;
//     cout<<"estimated a,b,c = ";
//     for ( auto a:abc ) cout<<a<<" ";
//     cout<<endl;

//     return 0;
// }


// #include <GL/glew.h>
// #include <pangolin/pangolin.h>

// int main( int /*argc*/, char** /*argv*/ )
// {
//     // 创建名称为“Main”的GUI窗口，尺寸为640×640
//     pangolin::CreateWindowAndBind("Main",640,480);
//     // 启动深度测试
//     glEnable(GL_DEPTH_TEST);

//     // 创建一个观察相机视图
//     pangolin::OpenGlRenderState s_cam(
//         pangolin::ProjectionMatrix(640,480,420,420,320,320,0.2,100),
//         pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
//     );

//     // 创建交互视图
//     pangolin::Handler3D handler(s_cam); //交互相机视图句柄
//     pangolin::View& d_cam = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
//             .SetHandler(&handler);

//     while( !pangolin::ShouldQuit() )
//     {
//         // 清空颜色和深度缓存
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);

//         // 在原点绘制一个立方体
//         pangolin::glDrawColouredCube();
//         // 绘制坐标系
//        	glLineWidth(3);
//         glBegin ( GL_LINES );
// 	    glColor3f ( 0.8f,0.f,0.f );
// 	    glVertex3f( -1,-1,-1 );
// 	    glVertex3f( 0,-1,-1 );
// 	    glColor3f( 0.f,0.8f,0.f);
// 	    glVertex3f( -1,-1,-1 );
// 	    glVertex3f( -1,0,-1 );
// 	    glColor3f( 0.2f,0.2f,1.f);
// 	    glVertex3f( -1,-1,-1 );
// 	    glVertex3f( -1,-1,0 );
// 	    glEnd();

//         // 运行帧循环以推进窗口事件
//         pangolin::FinishFrame();
//     }
    
//     return 0;
// }


// Eigen::MatrixXd getPx(){

//   return Px;
// }

/*
,
                "-I",
                "${workspaceFolder}/include",
                "-I",
                "/opt/Homebrew/include",
                "-L",
                "/opt/Homebrew/Cellar/glew/2.2.0_1/lib",
                "-L",
                "/opt/Homebrew/Cellar/glfw/3.3.4/lib",
                "-I",
                "/opt/homebrew/Cellar/eigen/3.3.9/include/eigen3",
                "-I",
                "/opt/homebrew/Cellar/boost/1.75.0_3/include"




                "-L",
                "/opt/Homebrew/Cellar/glew/2.2.0_1/lib",
                "-L",
                "/opt/Homebrew/Cellar/glfw/3.3.4/lib"




                "-l",
                "pangolin",
                "-L",
                "pango_windowing",
                "-l",
                "pango_video",
                "-l",
                "pango_vars",
                "-l",
                "pango_tools",
                "-l",
                "pango_scene",
                "-l",
                "pango_plot",
                "-l",
                "pango_packetstream",
                "-l",
                "pango_opengl",
                "-l",
                "pango_image",
                "-l",
                "pango_glgeometry",
                "-l",
                "pango_geometry",
                "-l",
                "pango_display",
                "-l",
                "pango_python",
                "-l",
                "pango_core",
                "-l",
                "tinyobj",



*/

