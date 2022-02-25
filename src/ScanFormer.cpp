#include "ScanFormer.h"

namespace CS_SLAM{
/*
ScanFormer类定义
目标：用得到的sonar帧形成一个full scan，需要用到各scan的位姿
*/
class KeyFrame;

ScanFormer::ScanFormer(EKF* mpEKF_):mpEKF(mpEKF_){
    scan.resize(0);
    x_s.resize(0);
}

ScanFormer::~ScanFormer(){}

std::vector<point> ScanFormer::GetFullScan(){
    return z;
}

const std::vector<Eigen::VectorXd>& ScanFormer::GetScan(){
    return scan;
}

void ScanFormer::BeamSegment(int thresh = 40){
    /*
    输入：beam的一轮完整scan, 强度阈值thresh
    输出(内隐)：阈值化并局部最大化后的scan
    */
    for(int i = 0; i < scan.size(); i++){
        // for(int j = 0; j < scan[i].size(); j++){
        //     if(scan[i](j) < thresh)scan[i](j) = 0;
        // }
        int localMax = INT_MIN;
        int localMaxID = -1;
        for(int j = 10; j < scan[i].size(); j++){
            if(scan[i](j) > localMax){
                localMax = scan[i](j);
                localMaxID = j;
            }
            // scan[i](j) = 0;
        }
        // scan[i](localMaxID) = localMax;
        if(localMax>=thresh){
            double r=0.0503*localMaxID, theta=2*PI/200*i;
            Eigen::Vector2d z_hat(r*cos(theta),r*sin(theta));
            Eigen::Matrix2d z_P = Eigen::Matrix2d::Zero(2,2);
            z.push_back(point(z_hat,z_P));
        }else{
            // std::cout<<"row: "<<i<<" no useful information"<<std::endl;
            continue;
        }
        // if(localMaxID != -1){
        //     double r=0.0503*localMaxID, theta=2*PI/200*i;
        //     Eigen::Vector2d z_hat(r*cos(theta),r*sin(theta));
        //     Eigen::Matrix2d z_P = Eigen::Matrix2d::Zero(2,2);
        //     z.push_back(point(z_hat,z_P));
        //     // std::cout<<"pushed "<<r<<","<<theta<<" and "<<z_hat<<std::endl;
        // }else{
        //     std::cout<<"row: "<<i<<" no useful information"<<std::endl;
        //     z.push_back(KeyFrame(Eigen::VectroXd(0,0,0),
        //                    Eigen::Zero(3,3)));//(0,0)表示无有效样本点
        // }
    }
    return ;
}

//把Ii平移到Ic
motion ScanFormer::D(int i){
    motion res;
    if(i == C) return res;
    else if(i > C){
        for(int j = C+1; j <= i; j++){
            Eigen::VectorXd x_ = x_s[j].GetPose().hat;
            Eigen::Vector3d d_(x_(0), x_(1), 0);
            res.hat = res.hat + d_;
        }
    }else{
        for(int j = i; j < C; j++){
            Eigen::VectorXd x_ = x_s[j].GetPose().hat;
            // std::cout<<"here2:"<<x_s[j].GetPose().hat<<std::endl;
            Eigen::Vector3d d_(x_(0), x_(1), 0);
            res.hat = res.hat - d_;
        }
    }
    return res;
}

bool ScanFormer::IsFull(){
    return sonarCnt == NUM_BEAMS;
}


//根据相对运动估计生成生成完整的scan
void ScanFormer::Undistort(int thresh=40){
    //输入(内隐)：scan
    //输出(内隐)：修正好的scan
    //先阈值化
    // std::cout<<"--Start Undistort"<<std::endl;
    // std::cout<<"----Start BeamSegment"<<std::endl;
    BeamSegment(thresh);//对该轮声纳Segment之后得到点云
    // std::cout<<"----End BeamSegment"<<std::endl;
    for(int i = 0;i < z.size(); i++){
        if(i == C)continue;
        // std::cout<<z[i].hat<<std::endl;
        motion r_(Eigen::Vector3d(0,0,x_s[i].GetPose().hat(2)), Eigen::Matrix3d::Zero(3,3));
        motion r_c(Eigen::Vector3d(0,0,x_s[C].GetPose().hat(2)), Eigen::Matrix3d::Zero(3,3));
        // std::cout<<"rc:\n"<<r_c.P<<"\nD(i):"<<D(i).P<<std::endl;
        //std::cout<<"------undistorting beam "<<i<<std::endl;
        // z[i] = (r_c.tail2tail(D(i))).compoundP(r_.compoundP(z[i]));
        z[i] = (r_c.tail2tail(x_s[i].GetPose())).compoundP(r_.compoundP(z[i]));
        // std::cout<<"after undistort"<< z[i].hat<<std::endl;
    }
    // std::cout<<"--End Undistort"<<std::endl;
    return ;
}
/**
 * @brief use DVL to update local pose
 * 
 * @param data_dvl 
 * @param dt 
 */
void ScanFormer::UseDVL(Eigen::VectorXd data_dvl, double dt){
    // if(!mpEKF->isInitialized()){
    //     return ;
    // }
    Eigen::MatrixXd H_DVL(3,8);
    H_DVL << 0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0;
            // 0,0,1,0,0,0,0,0;
    mpEKF->setH(H_DVL);
    mpEKF->setR(Eigen::MatrixXd::Identity(3,3));
    // std::cout<<"----start DVL update"<<std::endl;
    mpEKF->update(data_dvl, dt);
    // std::cout<<"----end DVL update"<<std::endl;
//DR
    // Eigen::VectorXd x_stmp = mpEKF->GetX();
    // Eigen::MatrixXd x_sptmp = mpEKF->GetP();
    // Eigen::VectorXd x_shat = Eigen::Vector3d(x_stmp(0),x_stmp(1),x_stmp(3));
    // Eigen::Matrix3d x_sp;
    // x_sp.block(0,0,2,2)=x_sptmp.block(0,0,2,2);x_sp.block(0,2,2,1)=x_sptmp.block(0,3,2,1);
    // x_sp.block(2,0,1,2)=x_sptmp.block(3,0,1,2);x_sp(2,2)=x_sptmp(3,3);
    // std::cout<<"pushing "<<x_shat<<std::endl;
    // x_s.push_back(KeyFrame(x_shat,x_sp));
}

void ScanFormer::UseAHRS(Eigen::VectorXd data_ahrs, double dt){
    // if(!mpEKF->isInitialized()){
    //     Eigen::VectorXd tmp=Eigen::VectorXd::Zero(8);
    //     tmp(4) = data_ahrs(0);
    //     mpEKF->initialize(tmp);
    // }
    Eigen::MatrixXd H_AHRS(1,8);
    H_AHRS << 0,0,0,1,0,0,0,0;
    mpEKF->setH(H_AHRS);
    mpEKF->setR(0.01*Eigen::MatrixXd::Identity(1,1));
    // std::cout<<"----start AHRS update"<<std::endl;
    mpEKF->update(data_ahrs, dt);
    // std::cout<<"----end AHRS update"<<std::endl;
//DR
    // Eigen::VectorXd x_stmp = mpEKF->GetX();
    // Eigen::MatrixXd x_sptmp = mpEKF->GetP();
    // Eigen::VectorXd x_shat = Eigen::Vector3d(x_stmp(0),x_stmp(1),x_stmp(3));
    // Eigen::Matrix3d x_sp;
    // x_sp.block(0,0,2,2)=x_sptmp.block(0,0,2,2);x_sp.block(0,2,2,1)=x_sptmp.block(0,3,2,1);
    // x_sp.block(2,0,1,2)=x_sptmp.block(3,0,1,2);x_sp(2,2)=x_sptmp(3,3);
    // std::cout<<"pushing "<<x_shat<<std::endl;
    // x_s.push_back(KeyFrame(x_shat,x_sp));
}

void ScanFormer::UseDS(Eigen::VectorXd data_ds, double dt){
    // if(!mpEKF->isInitialized()){
    //     return ;
    // }
    Eigen::MatrixXd H_DS(1,8);
    H_DS << 0,0,1,0,0,0,0,0;
    mpEKF->setH(H_DS);
    mpEKF->setR(0.01*Eigen::MatrixXd::Identity(1,1));
    // std::cout<<"----start DS update"<<std::endl;
    // std::cout<<"data_ds"<<data_ds<<" | dt:"<<dt<<std::endl;
    mpEKF->update(data_ds, dt);
    // std::cout<<"to be"<<mpEKF->GetX()<<std::endl;
    // std::cout<<"----end DS update"<<std::endl;
//DR
    // Eigen::VectorXd x_stmp = mpEKF->GetX();
    // Eigen::MatrixXd x_sptmp = mpEKF->GetP();
    // Eigen::VectorXd x_shat = Eigen::Vector3d(x_stmp(0),x_stmp(1),x_stmp(3));
    // Eigen::Matrix3d x_sp;
    // x_sp.block(0,0,2,2)=x_sptmp.block(0,0,2,2);x_sp.block(0,2,2,1)=x_sptmp.block(0,3,2,1);
    // x_sp.block(2,0,1,2)=x_sptmp.block(3,0,1,2);x_sp(2,2)=x_sptmp(3,3);
    // std::cout<<"pushing "<<x_shat<<std::endl;
    // x_s.push_back(KeyFrame(x_shat,x_sp));
}

void ScanFormer::UseSonar(Eigen::VectorXd data_sonar, double dt){
    /*
    目的：叠加声纳beam，并用其进行EKF的位姿估计。
    */
    //data_sonar.resize(0);
    // std::cout<<scan.size()<<std::endl;

    // std::cout<<data_sonar.size()<<std::endl;

    //scan[sonarCnt++] = data_sonar;
    // if(!mpEKF->isInitialized()){
    //     return ;
    // }
    scan.push_back(data_sonar);
    sonarCnt++;
    // std::cout<<"integrating"<<std::endl;
    //只在每轮reset一次
    if(sonarCnt == 1){
        mpEKF->ResetDeadReckoningXYZ();
        // std::cout<<"reseted"<<std::endl;
    }
    /*\每个beam都reset
    mpEKF->ResetDeadReckoningXYZ();
    */
    // std::cout<<"from x:\n"<<mpEKF->GetX()<<std::endl;
    mpEKF->prediction(dt);
    // std::cout<<"to x:\n"<<mpEKF->GetX()<<std::endl;

    // std::cout<<"----sonar_data "<< sonarCnt << " integrated into scan"<<std::endl;
    // std::cout<<"push x:\n"<<mpEKF->GetX()<<" P:"<<mpEKF->GetP()<<std::endl;
    //选取状态量中的x y psi作为x_s的元素
    Eigen::VectorXd x_stmp = mpEKF->GetX();
    Eigen::MatrixXd x_sptmp = mpEKF->GetP();
    Eigen::Vector3d x_shat = Eigen::Vector3d(x_stmp(0),x_stmp(1),x_stmp(3));
    Eigen::Matrix3d x_sp;
    x_sp.block(0,0,2,2)=x_sptmp.block(0,0,2,2);x_sp.block(0,2,2,1)=x_sptmp.block(0,3,2,1);
    x_sp.block(2,0,1,2)=x_sptmp.block(3,0,1,2);x_sp(2,2)=x_sptmp(3,3);
    // std::cout<<"pushing:"<<std::endl<<x_shat<<std::endl;
    x_s.push_back(KeyFrame(x_shat,x_sp));
    // std::cout<<"pushed"<<std::endl;
    if(sonarCnt == NUM_BEAMS){
        //扫完一轮就要矫正然后得到该轮图像以及轮内运动
        Undistort();
        // std::cout<<"scan是"<<scan.size()<<std::endl;
        // std::cout<<"z是"<<z.size()<<std::endl;
        // std::cout<<"x_s是"<<x_s.size()<<std::endl;
    }
}



motion ScanFormer::GetFullMotion(){
    // motion ans = D(NUM_BEAMS-1).tail2tail(D(0));
    motion ans = motion(x_s[NUM_BEAMS-1].GetPose().hat,x_s[NUM_BEAMS-1].GetPose().P);//该行表示x_s为一轮累积运动
    // motion ans = motion(x_s[0].GetPose().hat,x_s[0].GetPose().P).tail2tail(motion(x_s[NUM_BEAMS-1].GetPose().hat,x_s[NUM_BEAMS-1].GetPose().P));
    // motion ans = motion(x_s[NUM_BEAMS-1].GetPose().hat,x_s[NUM_BEAMS-1].GetPose().P);
    // for(int i=0;i<x_s.size();i++){
    //     std::cout<<"x_s "<<i<<":"<<x_s[i].GetPose().hat<<std::endl;
    // }
    
    // for(int i = 0; i < 5;i++){
    //     std::cout<<"see "<<i<<" "<<x_s[i].GetPose().hat<<std::endl;
    // }
    // <<D(0).hat<<" =? "<<x_s[0].GetPose().hat<<" vs "<<x_s[1].GetPose().hat;
    // std::cout<<"while "<<D(199).hat<<std::endl;

    // std::cout<<ans.hat<<std::endl;
    return ans;
}

void ScanFormer::Reset(){
    mpEKF->reset();
    x_s.resize(0);
    sonarCnt = 0;
    // std::cout<<"scan是"<<scan.size()<<std::endl;
    scan.resize(0);
    // std::cout<<"z是"<<z.size()<<std::endl;
    z.resize(0);
    // std::cout<<"x_s是"<<x_s.size()<<std::endl;
}


Eigen::VectorXd ScanFormer::getx_ss(){
    Eigen::VectorXd ans;
    ans.resize(x_s.size()*3);
    std::cout<<"tot x_s.size"<<x_s.size()<<std::endl;
    for(int i = 0; i< x_s.size();i++){
        ans.block(3*i,0,3,1) = x_s[i].GetPose().hat;
    }
    return ans;
}


}


// void BeamSegment(int thresh = 80){
//     /*
//     输入：beam的一轮完整scan, 强度阈值thresh
//     输出(内隐)：阈值化并局部最大化后的scan
//     */
//     for(int i = 0; i < scan.rows(); i++){
//         for(int j = 0; j < scan.cols(); j++){
//             if(scan(i,j) <= thresh)
//                 scan(i,j) = 0;
//             }
//             int localMax = INT_MIN;
//             int localMaxID = -1;
//             for(int j = 0; j < scan.cols(); j++){
//             if(scan(i,j) > localMax){
//                 localMax = scan(i,j);
//                 localMaxID = j;
//             }
//             scan(i,j) = 0;
//         }
//         scan(i, localMaxID) = localMax;
//     }
//     return ;
// }