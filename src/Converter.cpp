#include "Converter.h"

namespace CS_SLAM{


// static KeyFrame Converter::EKFX2X(Eigen::VectorXd hat, Eigen::MatrixXd p){
//     Eigen::VectorXd x_hat = Eigen::Vector3d(hat(0),hat(1),hat(3));
//     Eigen::Matrix3d x_p;
//     x_p.block(0,0,2,2)=p.block(0,0,2,2);x_p.block(0,2,2,1)=p.block(0,3,2,1);
//     x_p.block(2,0,1,2)=p.block(3,0,1,2);x_p(2,2)=p(3,3);
//     return KeyFrame(x_hat,x_p);
// }


}
