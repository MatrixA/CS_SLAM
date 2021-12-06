#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <limits.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/math/distributions.hpp>
#include <iterator>
#include <random>

#include "RandomVector.h"


namespace CS_SLAM{
//MSIS 30Hz, DVL 1.5Hz, AHRS 10Hz.

class EKF{
public:
    EKF();
    EKF(Eigen::VectorXd x);
    ~EKF();
    Eigen::VectorXd GetX();
    Eigen::MatrixXd GetP();

    void initialize(Eigen::VectorXd x_in);
    bool isInitialized();
    void reset();
    void ResetDeadReckoningXYZ();

    void setP(Eigen::MatrixXd P_in);
    void setQ(Eigen::MatrixXd Q_in);
    void setH(Eigen::MatrixXd H_in);
    void setR(Eigen::MatrixXd R_in);

    pose prediction(double dt);
    void update(Eigen::VectorXd z, double dt);
private:
    bool is_initialized_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

}

#endif // EKF_H
