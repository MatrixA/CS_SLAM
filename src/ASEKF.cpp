#include "ASEKF.h"


namespace CS_SLAM{


ASEKF::ASEKF(){
    is_initialized_ = false;
    X_ = Eigen::Vector3d(0,0,0);
    // std::cout<<"init OK"<<std::endl;
    //初始化在整个实验的起点，x,y,phi
}

ASEKF::ASEKF(Frames *pFramesDatabase):mpFramesDatabase(pFramesDatabase){}

ASEKF::~ASEKF(){}

void ASEKF::Initialize(motion kf_init){
    //只有\psi为读数，其他设为0，因为是要计算相对位移
    std::cout<<"---Start Initialize ASEKF"<<std::endl;
    X_ = kf_init.hat;
    P_ = kf_init.P;
    is_initialized_ = true;
    std::cout<<"---End Initialize ASEKF"<<std::endl;
}
bool ASEKF::IsInitialized(){
    return is_initialized_;
}
void ASEKF::reset(){
    P_.setZero();
    Q_.setZero();
    H_.setZero();
    R_.setZero();
    F_.setZero();
    X_.setZero();
    return ;
}
void ASEKF::SetP(Eigen::MatrixXd P_in){
    P_ = P_in;
}
void ASEKF::SetQ(Eigen::MatrixXd Q_in){
    Q_ = Q_in;
}
void ASEKF::SetH(Eigen::MatrixXd H_in){
    H_ = H_in;
}
void ASEKF::SetR(Eigen::MatrixXd R_in){
    R_ = R_in;
}

pose ASEKF::GetCurrentPose(){
    return pose(X_.head(3),P_.block(0,0,3,3));
}

void ASEKF::Prediction(motion q_n){
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
    AddToDatabase(pose(tmp_mu.block(0,0,3,1),0.1*Eigen::MatrixXd::Identity(3,3)));
    //X_.topRows(3)=Utils::Odot(X_.block(0,0,3,1), q_n.hat);

    /*方差*/
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

void ASEKF::Print(){
    std::cout<<"X_:"<<X_<<std::endl;
    std::cout<<"P:"<<P_<<std::endl;
}

void ASEKF::Update(int xi, int xn, motion ob){
    //scan matching结果作为实际观测进行更新
    Eigen::VectorXd z = ob.hat;
    Eigen::VectorXd y = z - H_ * X_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    X_ = X_ + (K*y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X_.size(),X_.size());
    P_ = (I - K * H_) * P_;
    return ;
}

void ASEKF::AddPose(KeyFrame xn){
    // Eigen::VectorXd X_new(X_.rows()+(xn.GetPos()).rows());
    // X_new.middleRows(0,3) = xn.GetPos();
    // X_new.middleRows(3,X_new.size()-3) = X_;
    // X_.resize(X_.rows()+xn.GetPos().size());
    int N = X_.rows();
    Eigen::MatrixXd tmp_mu(N+3, 1);
    Eigen::Vector3d a=X_.block(0,0,3,1);
    Eigen::Vector3d b=xn.GetPos();

    tmp_mu.block(0,0,3,1) = Eigen::Vector3d(a(0)+b(0),a(1)+b(1),b(2));
    tmp_mu.block(3,0,N,1) = X_;
    X_.resize(N+3, 1);
    X_ = tmp_mu;
    // X_.tail(X_.rows()-xn.GetPos().size()) = X_.head(X_.rows()-xn.GetPos().size());
    // std::cout<<"------Start Add a Pose"<<std::endl;
    // std::cout<<"------Adding a Pose then:\n"<<X_<<"\n xn:\n"<<xn.GetPos()<<std::endl;
    // X_.head(3) = xn.GetPos();
    // X_new << xn.GetPos(),X_;
    // X_ = X_new;
    // const int N = X_.rows();
    // Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(N,N);
    // P_new.block(0,0,3,3) = xn.GetPosP();
    // P_new.block(3,3,N-3,N-3) = P_;
    // P_ = P_new;
    // F_ = Eigen::MatrixXd::Identity(N, N);
    // F_(2,2)=0;
    // Eigen::MatrixXd F = Eigen::MatrixXd::Identity(N+3, N+3);F(2,2)=0;//TODO
    // Eigen::MatrixXd G = Eigen::MatrixXd(N+3, 3);G(0,0)=1;G(1,1)=1;G(2,2)=1;//TODO
    // Eigen::MatrixXd tmp = F*P_*F.transpose() + G * xn.GetPosP() * G.transpose();
    // P_.resize(N+3,N+3);
    // P_ = tmp;
    // std::cout<<"------End Add a Pose"<<std::endl;
    return ;
}


void ASEKF::AddToDatabase(motion q){
    mpFramesDatabase->add(KeyFrame(q));
    return ;
}


Eigen::VectorXd ASEKF::GetX(){
    return X_;
}

}
//得到一个scan_pose之后，该scan_pose和之前的scan_pose进行scanMatching，每次相邻结果都作为约束更新随机图。



