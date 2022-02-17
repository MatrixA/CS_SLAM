#ifndef MSCKF_H
#define MSCKF_H


#include <iostream>
#include <limits.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/math/distributions.hpp>
#include <iterator>
#include <random>
#include <ceres/ceres.h>

#include "RandomVector.h"
#include "KeyFrame.h"
#include "Frames.h"
#include "Utils.h"

namespace CS_SLAM{

class RandomVector;

class MSCKF{
public:
    MSCKF();
    ~MSCKF();
    MSCKF(Frames *pFramesDatabase);

    void Initialize(motion kf_init);
    bool IsInitialized();
    void reset();

    Eigen::VectorXd GetX();
    pose GetCurrentPose();
    pose AddPose(KeyFrame xn, const bool modify=true);

    void SetP(Eigen::MatrixXd P_in);
    void SetQ(Eigen::MatrixXd Q_in);
    void SetH(Eigen::MatrixXd H_in);
    void SetR(Eigen::MatrixXd R_in);
    void Print();

    void Prediction(motion q_n);
    void AddToDatabase(pose q);
    void Update(int xi, int xn, motion ob);
    void UpdateDatabase();//TODO:

private:
    bool is_initialized_;
    Eigen::VectorXd X_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Frames* mpFramesDatabase;

};

//得到一个scan_pose之后，该scan_pose和之前的scan_pose进行scanMatching，每次相邻结果都作为约束更新随机图。
}

#endif // MSCKF_H