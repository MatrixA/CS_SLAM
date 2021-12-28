#ifndef RANDOMVECTOR_H
#define RANDOMVECTOR_H

#include <iostream>
#include <limits.h>
#include <algorithm>
#include <Eigen/Core>
#include <boost/math/distributions.hpp>
#include <utility>


namespace CS_SLAM{
//_EIGEN_INITIALIZE_MATRICES_BY_ZERO或者EIGEN_INITIALIZE_MATRICES_BY_NAN
class RandomVector{

public:
    RandomVector();
    RandomVector(Eigen::VectorXd hat_, Eigen::MatrixXd P_);

    void Print();
    
    RandomVector compound(RandomVector b);

    RandomVector compoundP(RandomVector b);
    static RandomVector CompoundP(RandomVector q, RandomVector b);
    RandomVector rinverse();

    RandomVector tail2tail(RandomVector b);

    bool operator ==(const RandomVector& b);
    Eigen::VectorXd hat;
    Eigen::MatrixXd P;
};

typedef RandomVector point, pose, motion, error;

// Eigen::Vector3d compound(Eigen::Vector3d a, Eigen::Vector3d b){
// // /*
// // 输入：两个首尾相接的关系随机向量
// // 输出：两个关系随机向量的复合
// // */
// //    return Eigen::Vector3d(b(0)*cos(a(2))-b(1)*sin(a(2))+a(0),
// //                            b(0)*sin(a(2))+b(1)*cos(a(2))+a(1),
// //                            a(2)+b(2));
// // }

// Eigen::Vector3d sinverse(Eigen::Vector3d a){
// /*
// 输入：状态变化向量
// 输出：状态逆变化向量
// */
//    return Eigen::Vector3d(-a(0)*cos(a(2))-a(1)*sin(a(2)),
//                         a(0)*sin(a(2))-a(1)*cos(a(2)),
//                         -a(2));
// }

// Eigen::Vector3d tail2tail(Eigen::Vector3d a, Eigen::Vector3d b){
// /*
// 输入：两个状态向量a,b
// 输出：从状态a到b的状态变化向量
// */
//    return compound(sinverse(a),b);
// }


// struct MahResidual{
//     MahResidual(point sr,point sn):ai(sr),ci(sn){}
//     template <typename T>
//     bool operator()(const T* const q0,const T* const q1, const T* const q2, T* residual) const{
//     // bool operator()(const T* const abc, T* residual) const{
//         Eigen::Vector2d ei(ai.hat(0)-ci.hat(0)*cos(q2[0])+ci.hat(1)*sin(q2[0])-q0[0],
//                             ai.hat(1)-ci.hat(0)*sin(q2[0])-ci.hat(1)*cos(q2[0])-q1[0]);
//         Eigen::MatrixXd J_q, J_c;
//         J_q << 1, 0, -ci.hat(0)*sin(q2[0])-ci.hat(1)*cos(q2[0]),
//                0, 1, ci.hat(0)*cos(q2[0])+ci.hat(1)*sin(q2[0]);
//         J_c << double(cos(q2[0])), double(-sin(q2[0])),
//                 double(sin(q2[0])), double(-cos(q2[0]));
//         Eigen::MatrixXd P = J_q*J_q.transpose() + J_c*ci.P*J_c.transpose();
//         // MatrixXd P = /* rj.P + */ J_q*q.P*J_q.transpose() + J_c*ci.P*J_c.transpose();
//         // residual[0] = T(ei.transpose()*P.inverse()*ei);
//         residual[0] = T(ei.transpose()*P.inverse()*ei);
//         return true;
//     }
//     const point ai;
//     const point ci;
// };

// Eigen::Vector3d modpIC(std::vector<point> S_ref, std::vector<point> S_new, motion q, int maxIterations = 100){
// /*!!!!!!
//     输入：参考帧S_ref，新帧S_new，初始位移估计q(包含期望和方差)
//     输出：更优的位移估计(仅期望)
// */
//     int k = 0;
//     unsigned int m = S_new.size();
//     RandomVector qk;
//     qk.hat = q.hat;
//     qk.P = Eigen::MatrixXd::Zero(3,3);
//     // Eigen::Vector3d hat_qk = q.hat;
//     // Eigen::Vector3d q;
//     do{
//         if(k!=0)qk = q;
//         //找到S_new中每个点的匹配点(可能不存在)
//         std::vector<Eigen::Vector3d> hat_e[m];
//         Eigen::Vector3d hat_a;
//         std::vector<std::pair<int, int> >Match;
//         for(int i = 0; i<m; i++){
//             Eigen::Vector2d n_i=oplus(qk.hat,S_new[i].hat);
//             //找到S_ref中与n_i马氏距离小于X的点

//             double mindis = DBL_MAX;
//             int minid=-1;
//             boost::math::chi_squared mydist(2);
//             double thresh = boost::math::quantile(mydist, 0.2);
//             for(int j = 0; j < S_ref.size(); j++){
//                 Eigen::MatrixXd J_q, J_c;
//                 point ci = S_new[i];
//                 point rj = S_ref[j];
//                 J_q << 1, 0, -ci.hat(0)*sin(qk.hat(2))-ci.hat(1)*cos(qk.hat(2)),
//                     0, 1, ci.hat(0)*cos(qk.hat(2))+ci.hat(1)*sin(qk.hat(2));
//                 J_c << cos(qk.hat(2)), -sin(qk.hat(2)),
//                     sin(qk.hat(2)), -cos(qk.hat(2));
//                 Eigen::MatrixXd C = rj.P + J_q*q.P*J_q.transpose() + J_c*ci.P*J_c.transpose();

//                 double mahdis = MahDistance(S_ref[j].hat, C, n_i);
//                 if(mahdis < mindis && mahdis <= thresh){
//                     mindis = mahdis;
//                     minid = j;
//                 }
//             }
//             if(minid != -1){
//                 Match.push_back(std::make_pair(i,minid));
//             }
//         }

//         //对匹配点构建和求解优化问题
//         double q0 = 0.0;
//         double q1 = 0.0;
//         double q2 = 0.0;

//         ceres::Problem problem;
//         for(int i=0; i<Match.size(); i++){
//             point sr = S_ref[Match[i].first];
//             point sn = S_new[Match[i].second];
//             ceres::CostFunction* cost_function =
//             new ceres::AutoDiffCostFunction<MahResidual, 1, 1, 1, 1>(
//                 new MahResidual(sr,sn));
//             problem.AddResidualBlock(cost_function, nullptr, &q0, &q1, &q2);
//         }
//         ceres::Solver::Options options;
//         options.max_num_iterations = maxIterations;
//         options.linear_solver_type = ceres::DENSE_QR;
//         options.minimizer_progress_to_stdout = true;
//         ceres::Solver::Summary summary;
//         Solve(options, &problem, &summary);
//         q.hat << q0,q1,q2;
//         k++;
        
//         //    Eigen::Vector3d hat_a = minr;
//         //    hat_e[i] = hat_a - compound(hat_qk, hat_n[i]);
//         //    P_ei = P_ai + J_1*P_q*J_q.transpose()+J_c*P_ci*J_c.transpose()
//         // }
//         // hat_q_min = argmin_q;
//         // if(hat_q_min == hat_qk){//converge
//         //    return hat_q_min;
//         // }else{
//         //    hat_q = hat_qmin;
//         //    k ++;
//         // }
//     }while(!(q==qk) && k < maxIterations);
//     return q.hat;
// }




// motion ScanMatching(int Sn, int Si){
// //对两full scan做扫描匹配得到相对运动及其不确定性估计
//     motion d;
//     //获得初始位移估计hat_q
//     motion q = all_scan_pose[Sn].tail2tail(all_scan_pose[Si]);
//     //获得初始位移估计不确定性P_q
//     Eigen::Matrix<double, 3, 3> P_q;
//     Eigen::Matrix<double, 3, 3> H_k;
//     Eigen::Matrix<double, 3, 3> P_k;
//     P_q = H_k * P_k * H_k.transpose();
//     d.hat = modpIC(all_scan[Sn],all_scan[Si],motion(q.hat,q.P));
//     d.P = Eigen::Matrix3d::Zero(3,3);//TODO;
//     return d;
// }


}
//支持克罗内克积：kroneckerProduct(mat1，mat2)

// Eigen::MatrixXd getPq(){
// Eigen::MatrixXd Pq;
// Eigen::VectorXd R(2*k);

// Eigen::MatrixXd W;
// Eigen::MatrixXd L = W.llt().matrixL();




#endif