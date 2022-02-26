#include "MSCKF.h"


namespace CS_SLAM{


MSCKF::MSCKF(){
    is_initialized_ = false;
    X_ = Eigen::Vector3d(0,0,0);
    P_ = 0.001*Eigen::Matrix3d::Identity(3,3);
    R_ = 0.001*Eigen::Matrix3d::Identity(3,3);
    // std::cout<<"init OK"<<std::endl;
    //初始化在整个实验的起点，x,y,phi
}

MSCKF::MSCKF(Frames *pFramesDatabase):mpFramesDatabase(pFramesDatabase){
    is_initialized_ = false;
    X_ = Eigen::Vector3d(0,0,0);
    P_ = 0.001*Eigen::Matrix3d::Identity(3,3);
    R_ = 0.001*Eigen::Matrix3d::Identity(3,3);
}

MSCKF::~MSCKF(){}

void MSCKF::Initialize(motion kf_init){
    //只有\psi为读数，其他设为0，因为是要计算相对位移
    std::cout<<"---Start Initialize MSCKF"<<std::endl;
    X_ = kf_init.hat;
    P_ = kf_init.P;
    is_initialized_ = true;
    std::cout<<"---End Initialize MSCKF"<<std::endl;
}
bool MSCKF::IsInitialized(){
    return is_initialized_;
}
void MSCKF::reset(){
    P_.setZero();
    Q_.setZero();
    H_.setZero();
    R_.setZero();
    F_.setZero();
    X_.setZero();
    return ;
}
void MSCKF::SetP(Eigen::MatrixXd P_in){
    P_ = P_in;
}
void MSCKF::SetQ(Eigen::MatrixXd Q_in){
    Q_ = Q_in;
}
void MSCKF::SetH(Eigen::MatrixXd H_in){
    H_ = H_in;
}
void MSCKF::SetR(Eigen::MatrixXd R_in){
    R_ = R_in;
}

pose MSCKF::GetCurrentPose(){
    return pose(X_.head(3),P_.block(0,0,3,3));
}



void MSCKF::Prediction(motion q_n){
    /*
    位姿图更新。
    输入: 估计的运动during last scan
    输出(含): 位姿图预测
    */
    /*均值*/
    int N = X_.rows();
    Eigen::MatrixXd tmp_mu(N+3, 1);
    Eigen::Vector3d a=X_.block(0,0,3,1);
    Eigen::Vector3d b=q_n.hat;

    tmp_mu.block(0,0,3,1) = Eigen::Vector3d(a(0)+b(0),a(1)+b(1),b(2));
    tmp_mu.block(3,0,N,1) = X_;
    X_.resize(N+3, 1);
    X_ = tmp_mu;
    //TO Modify P:
    // AddToDatabase(pose(tmp_mu.block(0,0,3,1),0.1*Eigen::MatrixXd::Identity(3,3)));
    //X_.topRows(3)=Utils::Odot(X_.block(0,0,3,1), q_n.hat);

    /*方差*/
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(N+3,N+3);
    P.block(0,0,3,3) = q_n.P;P.block(3,3,N,N)=P_;
    P_ = P;
    // Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N+3, N+3);F(2,2)=0;//TODO
    // Eigen::MatrixXd G = Eigen::MatrixXd(N+3, 3);G(0,0)=1;G(1,1)=1;G(2,2)=1;//TODO
    // P_ = F*P_*F.transpose() + G * q_n.P * G.transpose();
    
// void Prediction(pose x_k, motion q_n){
//位姿图的更新
//估计的时候，只是把最后一个位置叠加了航位推算，然后扩展了估计状态。
// MatrixXd A_odot = MatrixXd::Identity(3,3);
// A_odot(2,2)=0;
// MatrixXd B_odot = MatrixXd::Identity(3,3);

// pose x_predict;
// x_predict.hat_q = odot(x_k.hat_x, q_n.hat_q);
// all_scan_pose.insert(0, x_predict.hat_q);

// //设定F_k和G_k
// Eigen::MatrixXd F_k=Eigen::MatrixXd::Identity(9, 9);
// F_k(2, 2)=0;
// Eigen::MatrixXd<int, 9, 3> G_k;
// G_k(0, 0)=1;G_k(1, 1)=1;G_k(2, 2)=1;

// P_knext = F_k * P_k * F_k.transpose() + G_k * q_n.P * G_k.transpose();
}

void MSCKF::Print(){
    std::cout<<"X_:"<<X_<<std::endl;
    std::cout<<"P:"<<P_<<std::endl;
}

void MSCKF::Update(int xi, int xn, motion ob){
    //scan matching结果作为实际观测进行更新
    int N = X_.size()/3;
    Eigen::VectorXd z = ob.hat;
    H_ = Eigen::MatrixXd::Zero(3, X_.size());
    std::cout<<"Update "<<xi<<" with "<<xn<<" = "<<X_.size()<<std::endl;
    Eigen::Matrix3d tmp;
    tmp<<-cos(X_(3*xi+2)), -sin(X_(3*xi+2)), (X_(3*xi)-X_(3*xn))*sin(X_(3*xi+2))+(X_(3*xn+1)-X_(3*xi+1))*sin(X_(3*xi+2)),
         sin(X_(3*xi+2)), -cos(X_(3*xi+2)), (X_(3*xi+1)-X_(3*xn+1))*sin(X_(3*xi+2))-(X_(3*xn)-X_(3*xi))*cos(X_(3*xi+2)),
         0,0,-1;
    H_.block(0,3*xi,3,3) = tmp;
    tmp<<cos(X_(3*xi+2)), sin(X_(3*xi+2)), 0,
          -sin(X_(3*xi+2)), cos(X_(3*xi+2)), 0,
          0,0,1;
    H_.block(0,3*xn,3,3) = tmp;
    Eigen::VectorXd y = z - H_ * X_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    X_ = X_ + (K*y);
    mpFramesDatabase->AlterPose(0, pose(X_.topRows(3), 0.1*Eigen::Matrix3d::Identity()));
    mpFramesDatabase->AlterPose(N-1-xi, pose(X_.middleRows(3*xi,3), 0.1*Eigen::Matrix3d::Identity()));
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X_.size(),X_.size());
    P_ = (I - K * H_) * P_;

    return ;
}

pose MSCKF::AddPose(KeyFrame xn, const bool modify){
    // Eigen::VectorXd X_new(X_.rows()+(xn.GetPose().hat).rows());
    // X_new.middleRows(0,3) = xn.GetPose().hat;
    // X_new.middleRows(3,X_new.size()-3) = X_;
    // X_.resize(X_.rows()+xn.GetPose().hat.size());
    int N = X_.rows();
    Eigen::MatrixXd tmp_mu(N+3, 1);
    Eigen::Vector3d a=X_.block(0,0,3,1);
    Eigen::Vector3d b=xn.GetPose().hat;

    tmp_mu.block(0,0,3,1) = Eigen::Vector3d(a(0)+b(0),a(1)+b(1),b(2));
    tmp_mu.block(3,0,N,1) = X_;
    if(modify){
        X_.resize(N+3, 1);
        X_ = tmp_mu;
    }
    return pose(tmp_mu,xn.GetPose().P);
    // X_.tail(X_.rows()-xn.GetPose().hat.size()) = X_.head(X_.rows()-xn.GetPose().hat.size());
    // std::cout<<"------Start Add a Pose"<<std::endl;
    // std::cout<<"------Adding a Pose then:\n"<<X_<<"\n xn:\n"<<xn.GetPose().hat<<std::endl;
    // X_.head(3) = xn.GetPose().hat;
    // X_new << xn.GetPose().hat,X_;
    // X_ = X_new;
    // const int N = X_.rows();
    // Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(N,N);
    // P_new.block(0,0,3,3) = xn.GetPose().P;
    // P_new.block(3,3,N-3,N-3) = P_;
    // P_ = P_new;
    // F_ = Eigen::MatrixXd::Identity(N, N);
    // F_(2,2)=0;
    // Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N+3, N+3);F(2,2)=0;//TODO
    // Eigen::MatrixXd G = Eigen::MatrixXd(N+3, 3);G(0,0)=1;G(1,1)=1;G(2,2)=1;//TODO
    // Eigen::MatrixXd tmp = F*P_*F.transpose() + G * xn.GetPose().P * G.transpose();
    // P_.resize(N+3,N+3);
    // P_ = tmp;
    // std::cout<<"------End Add a Pose"<<std::endl;
    // return ;
}


// void MSCKF::AddToDatabase(motion q){
//     mpFramesDatabase->add(KeyFrame(q));
//     return ;
// }


Eigen::VectorXd MSCKF::GetX(){
    return X_;
}

}
//得到一个scan_pose之后，该scan_pose和之前的scan_pose进行scanMatching，每次相邻结果都作为约束更新随机图。



