#include "RandomVector.h"

namespace CS_SLAM{
//_EIGEN_INITIALIZE_MATRICES_BY_ZERO或者EIGEN_INITIALIZE_MATRICES_BY_NAN


RandomVector::RandomVector(){
    hat = Eigen::Vector3d::Zero(3);
    P = Eigen::Matrix3d::Zero(3,3);
}

RandomVector::RandomVector(Eigen::VectorXd hat_, Eigen::MatrixXd P_):hat(hat_),P(P_){}
    
void RandomVector::Print(){
    std::cout<<"hat:"<<hat<<std::endl<<"P: "<<P<<std::endl;
    return ;
}

/** 
 * @brief compound two random vectors that are head to tail.
 * Note: this compound is about two pose random vector.
 * @param b - random vector to be compound with
 * @return compounded random vector
 */
RandomVector RandomVector::compound(RandomVector b){
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

/**
 * @brief compound 2D point random vector with this randomvector
 * 
 * @param b - 2D point to be compounded with
 * @return compounded 2d RandomVector 
 */
RandomVector RandomVector::compoundP(RandomVector b){
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

/**
 * @brief compound 2D point random vector with this randomvector
 * @param q - motion q
 * @param b - 2D point to be compounded with
 * @return compounded 2d RandomVector 
 */
RandomVector RandomVector::CompoundP(RandomVector q, RandomVector b){
    Eigen::VectorXd oi = q.hat;
    Eigen::VectorXd ip = b.hat;
    Eigen::VectorXd hat = Eigen::Vector2d(oi(0)+ip(0)*cos(oi(2))-ip(1)*sin(oi(2)),
                        oi(1)+ip(0)*sin(oi(2))+ip(1)*cos(oi(2)));
    Eigen::MatrixXd J1(2,3),J2(2,2);
    J1 << 1,0,-ip(1)*sin(oi(2))-ip(1)*cos(oi(2)),
            0,1,ip(0)*cos(oi(2))-ip(1)*sin(oi(2));
    J2 << cos(oi(2)), -sin(oi(2)),
            sin(oi(2)), cos(oi(2));
    // std::cout<<"compoundP \n"<<this->P;
    Eigen::Matrix<double, 2, 2> P = J1*q.P*J1.transpose()+J2*b.P*J2.transpose();
    std::cout<<"what's wrong"<<std::endl;
    RandomVector ret(hat,P);
    return ret;
}


/**
 * @brief compute inverse of the random vector transform
 * 
 * @return the inversed random vector
 */
RandomVector RandomVector::rinverse(){
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

/**
 * @brief calculate tail to tail transform about two tail to tail random vectors
 * 
 * @param b - the random vector to do with.
 * @return resultant random vector 
 */
RandomVector RandomVector::tail2tail(RandomVector b){
    return (this->rinverse()).compound(b);
}

/**
 * @brief get Isometry of 2D state
 * 
 * @return Eigen::Isometry3d 
 */
Eigen::Isometry3d RandomVector::toSE3(){
    assert(hat.size()==3);
    Eigen::Isometry3d ret;
    ret(0,0)=1;ret(0,1)=0;ret(0,2)=0;ret(0,3)=hat(0);
    ret(1,0)=0;ret(1,1)=cos(hat(2));ret(1,2)=sin(hat(2));ret(1,3)=hat(1);
    ret(2,0)=0;ret(2,1)=sin(hat(2));ret(2,2)=cos(hat(2));ret(2,3)=0;
    ret(3,0)=0;ret(3,1)=0;ret(3,2)=0;ret(3,3)=1;
    return ret;
}

/**
 * @brief check if two random vectors are equal
 * Note: use isApprox in Eigen
 * @param b - the random vector to be compared with
 * @return true - equal
 * @return false - not equal
 */
bool RandomVector::operator ==(const RandomVector& b){
    return (this->hat).isApprox(b.hat, 1e-5) && (this->P).isApprox(b.P);
}



}