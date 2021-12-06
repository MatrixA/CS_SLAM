#include "ScanFormer.h"

namespace CS_SLAM{
/*
ScanFormer类定义
目标：用得到的sonar帧形成一个full scan，需要用到各scan的位姿
*/
class KeyFrame;

ScanFormer::ScanFormer(){
    scan.resize(0);
}
ScanFormer::~ScanFormer(){}

void ScanFormer::BeamSegment(int thresh = 80){
    /*
    输入：beam的一轮完整scan, 强度阈值thresh
    输出(内隐)：阈值化并局部最大化后的scan
    */
    for(int i = 0; i < scan.size(); i++){
        for(int j = 0; j < scan[i].size(); j++){
            if(scan[i](j) <= thresh)scan[i](j) = 0;
        }
        int localMax = INT_MIN;
        int localMaxID = -1;
        for(int j = 0; j < scan[i].size(); j++){
            if(scan[i](j) > localMax){
                localMax = scan[i](j);
                localMaxID = j;
            }
            scan[i](j) = 0;
        }
        scan[i](localMaxID) = localMax;
        if(localMaxID != -1){
            Eigen::Vector2d z_hat(0.1*localMaxID*cos(365/200*i),0.1*localMaxID*sin(365/200*i));
            Eigen::Matrix2d z_P = Eigen::Matrix2d::Zero(2,2);
            z.push_back(point(z_hat,z_P));
            std::cout<<z_hat<<std::endl;
        }else{
            std::cout<<"row: "<<i<<" no useful information"<<std::endl;
        //     z.push_back(KeyFrame(Eigen::VectroXd(0,0,0),
        //                    Eigen::Zero(3,3)));//(0,0)表示无有效样本点
        }
    }
    return ;
}

//把Ii平移到Ic
motion ScanFormer::D(int i){
    motion res;
    if(i >= C){
        for(int j = C; j <= i; j++){
            Eigen::VectorXd x_ = x_s[j].GetPos();
            Eigen::Vector3d d_(x_(0), x_(1), 0);
            res.hat = res.hat + d_;
        }
    }else{
        for(int j = i; j <= C; j++){
            Eigen::VectorXd x_ = x_s[j].GetPos();
            // std::cout<<"here2:"<<x_s[j].GetPos()<<std::endl;
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
void ScanFormer::Undistort(int thresh=80){
    //输入(内隐)：scan
    //输出(内隐)：修正好的scan
    //先阈值化
    std::cout<<"--Start Undistort"<<std::endl;
    std::cout<<"----Start BeamSegment"<<std::endl;
    BeamSegment(thresh);//对该轮声纳Segment之后得到点云
    std::cout<<"----End BeamSegment"<<std::endl;
    for(int i = 0;i < z.size(); i++){
        if(i == C)continue;
        motion r_(Eigen::Vector3d(0,0,x_s[i].GetPos()(2)), Eigen::Matrix3d::Zero(3,3));
        motion r_c(Eigen::Vector3d(0,0,x_s[C].GetPos()(2)), Eigen::Matrix3d::Zero(3,3));
        // std::cout<<"rc:\n"<<r_c.P<<"\nD(i):"<<D(i).P<<std::endl;
        //std::cout<<"------undistorting beam "<<i<<std::endl;
        z[i] = (r_c.tail2tail(D(i))).compoundP(r_.compoundP(z[i]));
    }
    std::cout<<"--End Undistort"<<std::endl;
    return ;
}

void ScanFormer::UseDVL(Eigen::VectorXd data_dvl, double dt,Eigen::VectorXd paramDVL){
    Eigen::MatrixXd H_DVL(3,8);
    H_DVL << 0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0;
            // 0,0,1,0,0,0,0,0;
    scanEKF.setH(H_DVL);
    scanEKF.setR(0.01*Eigen::MatrixXd::Identity(3,3));
    // std::cout<<"----start DVL update"<<std::endl;
    scanEKF.update(data_dvl, dt);
    // std::cout<<"----end DVL update"<<std::endl;

}

void ScanFormer::UseAHRS(Eigen::VectorXd data_ahrs, double dt,Eigen::VectorXd paramAHRS){
    Eigen::MatrixXd H_AHRS(1,8);
    H_AHRS << 0,0,0,1,0,0,0,0;
    scanEKF.setH(H_AHRS);
    scanEKF.setR(0.001*Eigen::MatrixXd::Identity(1,1));
    // std::cout<<"----start AHRS update"<<std::endl;
    scanEKF.update(data_ahrs, dt);
    // std::cout<<"----end AHRS update"<<std::endl;
}

void ScanFormer::UseSonar(Eigen::VectorXd data_sonar, double dt,Eigen::VectorXd paramSonar){
    /*
    目的：叠加声纳beam，并用其进行EKF的位姿估计。
    */
    //data_sonar.resize(0);
    //std::cout<<"resizing"<<std::endl;
    //scan[sonarCnt++] = data_sonar;
    scan.push_back(data_sonar);
    sonarCnt++;

    //std::cout<<"integrating"<<std::endl;
    if(sonarCnt == 1){
        scanEKF.ResetDeadReckoningXYZ();
    }
    
    scanEKF.prediction(dt);
    std::cout<<"----sonar_data "<< sonarCnt << " integrated into scan"<<std::endl;

    // std::cout<<"push x:\n"<<scanEKF.GetX()<<" P:"<<scanEKF.GetP()<<std::endl;
    Eigen::VectorXd x_stmp = scanEKF.GetX();
    Eigen::MatrixXd x_sptmp = scanEKF.GetP();
    Eigen::VectorXd x_shat = Eigen::Vector3d(x_stmp(0),x_stmp(1),x_stmp(3));
    Eigen::Matrix3d x_sp;
    x_sp.block(0,0,2,2)=x_sptmp.block(0,0,2,2);x_sp.block(0,2,2,1)=x_sptmp.block(0,3,2,1);
    x_sp.block(2,0,1,2)=x_sptmp.block(3,0,1,2);x_sp(2,2)=x_sptmp(3,3);
    //std::cout<<"pushing"<<std::endl;
    x_s.push_back(KeyFrame(x_shat,x_sp));
    //std::cout<<"pushed"<<std::endl;
    if(sonarCnt == NUM_BEAMS){
        //扫完一轮就要矫正然后得到该轮图像以及轮内运动
        Undistort();
        sonarCnt = 0;
        // std::cout<<"scan是"<<scan.size()<<std::endl;
        scan.resize(0);
        // std::cout<<"z是"<<z.size()<<std::endl;
        z.resize(0);
        // std::cout<<"x_s是"<<x_s.size()<<std::endl;
        x_s.resize(0);
    }
}

void ScanFormer::UseDS(Eigen::VectorXd data_ds, double dt,Eigen::VectorXd paramDS){
    Eigen::MatrixXd H_DS(1,8);
    H_DS << 0,0,1,0,0,0,0,0;
    scanEKF.setH(H_DS);
    scanEKF.setR(0.01*Eigen::MatrixXd::Identity(1,1));
    // std::cout<<"----start DS update"<<std::endl;
    scanEKF.update(data_ds, dt);
    // std::cout<<"----end DS update"<<std::endl;

}

KeyFrame ScanFormer::GetPose(){
    return x_s.back();
}

void ScanFormer::Reset(){
    scanEKF.reset();
    // scan.setZero();
    //x_s.resize(0);
    sonarCnt = 0;
}

void ScanFormer::DrawFullScan(){
    glBegin(GL_POINTS);
    for (int i=0;i<scan.size();i++){
        for(int j=0;j<scan[i].rows();j++){
            glColor3f(0, 0, 0);
            double theta=0.0503*i;
            double r=0.2*j;
            glVertex3d(r*cos(theta), r*sin(theta), scan[i](j));
        }
    }
    glEnd();
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