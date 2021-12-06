#include "RandomVector.h"

namespace CS_SLAM{
//_EIGEN_INITIALIZE_MATRICES_BY_ZERO或者EIGEN_INITIALIZE_MATRICES_BY_NAN


RandomVector::RandomVector(){
    hat = Eigen::Vector3d::Zero(3);
    P = Eigen::Matrix3d::Zero(3,3);
}
RandomVector::RandomVector(Eigen::VectorXd hat_, Eigen::MatrixXd P_):hat(hat_),P(P_){}
    
RandomVector RandomVector::compound(RandomVector b){
/*
输入：两个首尾相接的关系随机向量
输出：两个关系随机向量的复合
*/
    Eigen::VectorXd ij = this->hat;
    Eigen::VectorXd jk = b.hat;
    this->hat = Eigen::Vector3d(jk(0)*cos(ij(2))-jk(1)*sin(ij(2))+ij(0),
                        jk(0)*sin(ij(2))+jk(1)*cos(ij(2))+ij(1),
                        ij(2)+jk(2));
    Eigen::VectorXd ik = this->hat;
    Eigen::MatrixXd J1(3,3),J2(3,3);
    J1 << 1,0,-(ik(1)-ij(1)),
            0,1,(ik(0)-ij(0)),
            0,0,1;
    J2 << cos(ij(2)), -sin(ij(2)),0,
            sin(ij(2)), cos(ij(2)),0,
            0,0,1;
    this->P = J1*this->P*J1.transpose()+J2*b.P*J2.transpose();
    return *this;
}

RandomVector RandomVector::compoundP(RandomVector b){
/*
输入：两个首尾相接的关系随机向量
输出：两个关系随机向量的复合
*/
    Eigen::VectorXd oi = this->hat;
    Eigen::VectorXd ip = b.hat;
    this->hat = Eigen::Vector2d(oi(0)+ip(0)*cos(oi(2))-ip(1)*sin(oi(2)),
                        oi(1)+ip(0)*sin(oi(2))+ip(1)*cos(oi(2)));
    Eigen::MatrixXd J1(2,3),J2(2,2);
    J1 << 1,0,-ip(1)*sin(oi(2))-ip(1)*cos(oi(2)),
            0,1,ip(0)*cos(oi(2))-ip(1)*sin(oi(2));
    J2 << cos(oi(2)), -sin(oi(2)),
            sin(oi(2)), cos(oi(2));
    // std::cout<<"compoundP \n"<<this->P;
    this->P = J1*this->P*J1.transpose()+J2*b.P*J2.transpose();
    return *this;
}

RandomVector RandomVector::rinverse(){
/*
输入：关系随机向量
输出：逆关系随机向量
*/
    Eigen::VectorXd x = this->hat;
    this->hat << -x(0)*cos(x(2))-x(1)*sin(x(2)),x(0)*sin(x(2))-x(1)*cos(x(2)),-x(2);
    Eigen::MatrixXd J(3,3);
    J<< -cos(x(2)),-sin(x(2)),x(1),
        sin(x(2)),-cos(x(2)),-x(0),
        0,0,-1;
    this-> P = J*this->P*J.transpose();
    // std::cout<<"reverse OK"<<std::endl;
    return *this;
}

RandomVector RandomVector::tail2tail(RandomVector b){
    return (this->rinverse()).compound(b);
}

bool RandomVector::operator ==(const RandomVector& b){
    return (this->hat).isApprox(b.hat, 1e-5) && (this->P).isApprox(b.P);
}



}