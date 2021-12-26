#include "EKF.h"

//MSIS 30Hz, DVL 1.5Hz, AHRS 10Hz.
//state vector: (x y z theta u v w r)^T

namespace CS_SLAM{
EKF::EKF(){
    x_=Eigen::VectorXd::Zero(8);
    F_=Eigen::MatrixXd::Identity(8,8);
    P_=Eigen::MatrixXd::Identity(8,8);
}
EKF::EKF(const Eigen::VectorXd x):x_(x){
    is_initialized_ = false;
    P_=0.01*Eigen::MatrixXd::Identity(8,8);
}
EKF::~EKF(){}
void EKF::initialize(const Eigen::VectorXd x_in){
    //只有\psi为读数，其他设为0，因为是要计算相对位移
    x_ = x_in;
    is_initialized_ = true;
}
bool EKF::isInitialized(){
    return is_initialized_;
}
void EKF::reset(){
    P_ = Eigen::MatrixXd::Identity(8,8);
    Q_ = 0.01*Eigen::MatrixXd::Identity(8,8);
    H_.setZero();
    R_ = 0.01*Eigen::MatrixXd::Identity(8,8);
    F_.setZero();
    x_.setZero();
    return ;
}
void EKF::ResetDeadReckoningXYZ(){
    //在新一轮扫描开始的时候，将EKF中的坐标重置为0
    x_(0)=0;
    x_(1)=0;
    x_(2)=0;
}
void EKF::setP(const Eigen::MatrixXd P_in){
    P_ = P_in;
}
void EKF::setQ(const Eigen::MatrixXd Q_in){
    Q_ = Q_in;
}
void EKF::setH(const Eigen::MatrixXd H_in){
    H_ = H_in;
}
void EKF::setR(const Eigen::MatrixXd R_in){
    R_ = R_in;
}
pose EKF::prediction(const double dt){
/*
输入：时间差dt
输出：预测的dt后的位姿
*/
    //每次位置和它的不确定性都设为0
    //得到的就是相对位移和不确定性
    // x_(0)=0;x_(1)=0;x_(2)=0;
    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(8,8);

    // P_new.block(3,3,x_.rows()-3,x_.rows()-3) = P_.block(3,3,x_.rows()-3,x_.rows()-3);
    // P_ = P_new;
    Eigen::Matrix<double, 4, 4> R;
    R << cos(x_(3)), -sin(x_(3)), 0, 0,
            sin(x_(3)), cos(x_(3)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    // const double mean=0.0, stddev=0.1;
    // std::default_random_engine generator;
    // std::normal_distribution<double> dist(mean, stddev);
    // double nk0=dist(generator), nk1=dist(generator),nk2=dist(generator), nk3=dist(generator);
    // std::cout<<nk0<<" "<<nk1<<" "<<nk2<<" "<<nk3<<std::endl;
    x_.middleRows(0,4) = x_.middleRows(0,4)+R*(dt*x_.middleRows(4,4));

    // x_next<< x_(0)+cos(x_(3))*(x_(4)*dt+nk0*dt*dt/2)-sin(x_(3))*(x_(5)*dt+nk1*dt*dt/2), x_(1)+sin(x_(3))*(x_(4)*dt+nk0*dt*dt/2)+cos(x_(3))*(x_(5)*dt+nk1*dt*dt/2), 
    //         x_(2)+x_(6)*dt+nk2*dt*dt/2, x_(3)+x_(7)*dt+nk3*dt*dt/2,
    //         x_(4)+nk0*dt, x_(5)+nk1*dt, x_(6)+nk2*dt, x_(7)+nk3*dt;
    // x_ = x_next;
    Eigen::Matrix<double, 8, 4>W;    
    W << cos(x_(3))*dt*dt/2,-sin(x_(3))*dt*dt/2,0,0,
         sin(x_(3))*dt*dt/2,cos(x_(3))*dt*dt/2,0,0,
         0,0,dt*dt/2,0,
         0,0,0,dt*dt/2,
         dt,0,0,0,
         0,dt,0,0,
         0,0,dt,0,
         0,0,0,dt;
    // std::cout<<"------start predicting"<<std::endl;
    F_ << 1,0,0,-sin(x_(3))*(x_(4)*dt)-cos(x_(3))*(x_(5)*dt), cos(x_(3))*dt, -sin(x_(3))*dt, 0, 0,
            0,1,0,cos(x_(3))*(x_(4)*dt)-sin(x_(3))*(x_(5)*dt), sin(x_(3))*dt, cos(x_(3))*dt, 0, 0,
            0,0,1,0,0,0,dt,0,
            0,0,0,1,0,0,0,dt,
            0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,1;
    
    Q_ =0.0001*Eigen::MatrixXd::Identity(4,4);
    P_ = F_ * P_ * F_.transpose()+ W*Q_*W.transpose();
    // std::cout<<"------predictingEKF over"<<std::endl;
    return pose(x_, P_);
}
// Eigen::VectorXd ekfPrediction(Eigen::VectorXd x_k, double dt){
// /*
// 输入：当前状态向量x_k, 时间间隔dt
// 输出：预测的状态向量
// */
//     Eigen::Matrix<double, 4, 4> R;
//     R << cos(x_k(3)), -sin(x_k(3)), 0, 0,
//             sin(x_k(3)), cos(x_k(3)), 0, 0,
//             0, 0, 1, 0,
//             0, 0, 0, 1;
//     const double mean=0.0, stddev=0.1;
//     std::default_random_engine generator;
//     std::normal_distribution<double> dist(mean, stddev);
//     double nk0=dist(generator), nk1=dist(generator),nk2=dist(generator), nk3=dist(generator);
//     cout<<nk0<<" "<<nk1<<" "<<nk2<<" "<<nk3<<endl;
//     Eigen::VectorXf x_knext(8);
//     x_knext<< x_k(0)+cos(x_k(3))*(x_k(4)*dt+nk0*dt*dt/2)-sin(x_k(3))*(x_k(5)*dt+nk1*dt*dt/2), x_k(1)+sin(x_k(3))*(x_k(4)*dt+nk0*dt*dt/2)+cos(x_k(3))*(x_k(5)*dt+nk1*dt*dt/2), 
//             x_k(2)+x_k(6)*dt+nk2*dt*dt/2, x_k(3)+x_k(7)*dt+nk3*dt*dt/2, 
//             x_k(4)+nk0*dt, x_k(5)+nk1*dt, x_k(6)+nk2*dt, x_k(7)+nk3*dt;
//     return x_knext;
// }

void EKF::update(const Eigen::VectorXd z, const double dt){
    prediction(dt);
    // std::cout<<"------start EKF update"<<std::endl;
    // std::cout<<"------cal y,z: "<<z<<"\n  H_:"<<H_<<"\n  x_:"<<x_<<std::endl;
    Eigen::VectorXd y = z - H_ * x_;
    
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K*y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(),x_.size());
    P_ = (I - K * H_) * P_;
    // std::cout<<"------end EKF update"<<std::endl;
}


Eigen::VectorXd EKF::GetX(){
    return x_;
}

Eigen::MatrixXd EKF::GetP(){
    return P_;
}

}


