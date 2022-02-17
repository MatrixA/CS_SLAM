#ifndef SCANFORMER_H
#define SCANFORMER_H

#include <pangolin/pangolin.h>
#include "MeasurementPackage.h"
#include "KeyFrame.h"
#include "EKF.h"
#include "Converter.h"


#define FSCAN_SIZE 20
#define NUM_BEAMS 200
#define NUM_BINS 397
#define PI acos(-1)


namespace CS_SLAM{
/*
ScanFormer类定义
目标：用得到的sonar帧形成一个full scan，需要用到各scan的位姿
*/
class KeyFrame;
class EKF;
class ScanFormer{
public:
    ScanFormer();
    ScanFormer(EKF* mpEKF_);
    ~ScanFormer();
    
    //重置ScanFormer
    void Reset();
    bool IsFull();

    std::vector<point> GetFullScan();
    const std::vector<Eigen::VectorXd>& GetScan();

    void BeamSegment(int thresh);
    //把Ii平移到Ic
    motion D(int i);
    //根据相对运动估计生成生成完整的scan
    void Undistort(int thresh);

    //使用DVL数据更新EKF
    void UseDVL(Eigen::VectorXd data_dvl, double dt);
    //使用AHRS数据更新EKF
    void UseAHRS(Eigen::VectorXd data_ahrs, double dt);
    //使用Sonar数据更新EKF并形成full scan
    void UseSonar(Eigen::VectorXd data_sonar, double dt);
    //使用DS数据更新EKF
    void UseDS(Eigen::VectorXd data_ds, double dt);



    Eigen::VectorXd getx_ss();

    //获得EKF的当前位姿(一般只在扫描一轮结束后调用)
    motion GetFullMotion();
    // void ScanFormer::DrawFullScan();

private:
    int sonarCnt=0;
    std::vector<Eigen::VectorXd> scan; //一个full_scan是NUM_BEAMS * NUM_BINS的强度矩阵
    EKF* mpEKF;
    // Eigen::Matrix<int, NUM_BEAMS, NUM_BINS> scan; //一个full_scan是NUM_BEAMS * NUM_BINS的强度矩阵
    std::vector<point> z; //从full_scan能够得到扫描点,z各点坐标为x,y
    std::vector<KeyFrame> x_s; //声纳full_scan的每帧位姿
    const int C = NUM_BEAMS/2;

};


}

#endif