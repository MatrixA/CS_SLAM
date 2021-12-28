#ifndef ASEKF_H
#define ASEKF_H


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
#include "Utils.h"

namespace CS_SLAM{

class RandomVector;

class ASEKF{
public:
    ASEKF();
    ~ASEKF();

    void Initialize(motion kf_init);
    bool IsInitialized();
    void reset();

    Eigen::MatrixXd GetX();
    void AddPose(KeyFrame xn);

    void SetP(Eigen::MatrixXd P_in);
    void SetQ(Eigen::MatrixXd Q_in);
    void SetH(Eigen::MatrixXd H_in);
    void SetR(Eigen::MatrixXd R_in);
    void Print();

    void Prediction(motion q_n);
    void Update(Eigen::VectorXd z);

private:
    bool is_initialized_;
    Eigen::VectorXd X_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

//得到一个scan_pose之后，该scan_pose和之前的scan_pose进行scanMatching，每次相邻结果都作为约束更新随机图。
}

#endif // ASEKF_H