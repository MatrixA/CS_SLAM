#include "LoopClosing.h"


namespace CS_SLAM
{

LoopClosing::LoopClosing(Frames* KeyFrameDatabase):mpKeyFrameDatabase(KeyFrameDatabase){}
LoopClosing::~LoopClosing(){}
// void LoopClosing::AddKeyFrame(KeyFrame* pKF){
//     keyFrames.push_back(pKF);
// }

// std::vector<KeyFrame *> LoopClosing::GetKeyFrames(){
//     return keyFrames;
// }

// std::vector<MapPoint *> LoopClosing::GetMapPoints(){
//     return mapPoints;
// }


class MahResidual{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MahResidual(const point& sr,const point& sn):ai(sr),ci(sn){}
    template <typename T>
    bool operator()(const T* const q0,const T* const q1, const T* const q2, T* residual) const{
        Eigen::Matrix<T,2,1>ei(T(ai.hat(0))-T(ci.hat(0))*ceres::cos(*q2)+T(ci.hat(1))*ceres::sin(*q2)-(*q0),
                               T(ai.hat(1))-T(ci.hat(0))*ceres::sin(*q2)-T(ci.hat(1))*ceres::cos(*q2)-(*q1));
        // Eigen::Vector2d ei(ai.hat(0)-ci.hat(0)*cos(q2)+ci.hat(1)*sin(q2)-q0,
        //                     ai.hat(1)-ci.hat(0)*sin(q2)-ci.hat(1)*cos(q2)-q1);
        Eigen::Matrix<T,2,3>J_q;
        J_q(0,0)=T(1); J_q(0,1)=T(0); J_q(0,2)=T(-1)*T(ci.hat(0))*ceres::sin(*q2)-T(ci.hat(1))*ceres::cos(*q2);
        J_q(1,0)=T(0); J_q(1,1)=T(1); J_q(1,2)=T(ci.hat(0))*ceres::cos(*q2)+T(ci.hat(1))*ceres::sin(*q2);
        // J_q<<1, 0, T(-1)*T(ci.hat(0))*ceres::sin(*q2)-T(ci.hat(1))*ceres::cos(*q2),
        //      0, 1, T(ci.hat(0))*ceres::cos(*q2)+T(ci.hat(1))*ceres::sin(*q2);
        Eigen::Matrix<T,2,2>J_c;
        J_c(0,0)=ceres::cos(*q2);J_c(0,1)=T(-1)*ceres::sin(*q2);
        J_c(1,0)=ceres::sin(*q2);J_c(1,1)=T(-1)*ceres::cos(*q2);
        Eigen::Map<Eigen::Matrix<T, 1, 1>> error{ residual };
        // Eigen::MatrixXd J_q(2,3), J_c(2,2);
        // J_q << 1, 0, -ci.hat(0)*sin(q2)-ci.hat(1)*cos(q2),
        //         0, 1, ci.hat(0)*cos(q2)+ci.hat(1)*sin(q2);
        // J_c << cos(q2), -sin(q2),
        //         sin(q2), -cos(q2);
        Eigen::Matrix<T,2,2> P = J_q*J_q.transpose() + J_c*(ci.P.template cast<T>())*J_c.transpose();
        // MatrixXd P = /* rj.P + */ J_q*q.P*J_q.transpose() + J_c*ci.P*J_c.transpose();
        error.template head<1>(1)=ei.transpose()*P.inverse()*ei;
        // residual[0]=T(ei.transpose()*P.conjugate()*ei);
        // residual[0] = ei.transpose()*P.inverse()*ei;
        return true;
    }
    static ceres::CostFunction* create(const point& a,const point& b){
        return (new ceres::AutoDiffCostFunction<MahResidual, 1, 1 ,1, 1>(new MahResidual(a,b)));
    }
private:
    point ai;
    point ci;
};


/**
 * @brief calculate motion from S_ref's view to S_new's view
 * 
 * @param S_ref : sonar full scan history
 * @param S_new : current sonar full scan
 * @param q : initial motion estimate 
 * @param maxIterations : max Iterations during optimization
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d modpIC(std::vector<point> S_new, std::vector<point> S_ref, motion q, int maxIterations = 5){
/*!!!!!!
    输入：参考帧S_ref，新帧S_new，初始位移估计q(包含期望和方差)
    输出：更优的位移估计(仅期望)
*/
    int k = 0;
    unsigned int m = S_new.size();
    motion qk;
    qk.hat = q.hat;
    qk.P = q.P;
    // Eigen::Vector3d hat_qk = q.hat;
    // Eigen::Vector3d q;
    std::cout<<"start to cycle"<<std::endl;
    do{
        if(k!=0)qk = q;
        //找到S_new中每个点的匹配点(可能不存在)
        // std::vector<Eigen::Vector3d> hat_e[m];
        // Eigen::Vector3d hat_a;
        std::vector<std::pair<int, int> >Match;
        std::cout<<"start to match, m="<<m<<std::endl;
        for(int i = 0; i<m; i++){
            std::cout<<"here is qk and p"<<std::endl;
            qk.Print();
            RandomVector::CompoundP(qk,S_new[i]).Print();
            Eigen::Vector2d n_i=(RandomVector::CompoundP(qk,S_new[i])).hat;
            //找到S_ref中与n_i马氏距离小于X的点
            std::cout<<"point "<<i<<":"<<S_new[i].hat<<" with motion "<<qk.hat << "to be "<<n_i<<std::endl;
            double mindis = DBL_MAX;
            int minid=-1;
            boost::math::chi_squared mydist(2);
            double thresh = boost::math::quantile(mydist, 0.2);
            for(int j = 0; j < S_ref.size(); j++){
                Eigen::MatrixXd J_q(2,3), J_c(2,2);
                point ci = S_new[i];
                point rj = S_ref[j];
                J_q << 1, 0, -ci.hat(0)*sin(qk.hat(2))-ci.hat(1)*cos(qk.hat(2)),
                    0, 1, ci.hat(0)*cos(qk.hat(2))+ci.hat(1)*sin(qk.hat(2));
                J_c << cos(qk.hat(2)), -sin(qk.hat(2)),
                    sin(qk.hat(2)), -cos(qk.hat(2));
                std::cout<<" with ref "<<j<<std::endl;

                Eigen::MatrixXd C = rj.P + J_q*q.P*J_q.transpose() + J_c*ci.P*J_c.transpose();
                std::cout<<"calc mahdis"<<std::endl;
                double mahdis = Utils::MahDistance(rj.hat, C, n_i);
                if(mahdis < mindis && mahdis <= thresh){
                    mindis = mahdis;
                    minid = j;
                }
            }
            if(minid != -1){
                Match.push_back(std::make_pair(i,minid));
            }
        }
        std::cout<<"start to optimize"<<std::endl;
        //对匹配点构建和求解优化问题
        double q0 = q.hat(0);
        double q1 = q.hat(1);
        double q2 = q.hat(2);
        std::cout<<"tot "<<Match.size()<<" matches"<<std::endl;
        ceres::Problem problem;
        for(int i=0; i<Match.size(); i++){
            point sr = S_ref[Match[i].first];
            point sn = S_new[Match[i].second];
            std::cout<<"add residual block "<<i<<": point"<<Match[i].first<<"with point"<<Match[i].second<<std::endl;
            // sr.Print();
            // sn.Print();
            ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
            ceres::CostFunction* cost_function =
            MahResidual::create(sr,sn);
            problem.AddResidualBlock(cost_function, loss_function, &q0, &q1, &q2);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = maxIterations;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        std::cout<<"start a solving"<<std::endl;
        Solve(options, &problem, &summary);
        std::cout<<"end a solving"<<std::endl;
        q.hat << q0,q1,q2;
        // std::cout<<"now q.hat"<<q.hat<<std::endl;
        k++;
        //    Eigen::Vector3d hat_a = minr;
        //    hat_e[i] = hat_a - compound(hat_qk, hat_n[i]);
        //    P_ei = P_ai + J_1*P_q*J_q.transpose()+J_c*P_ci*J_c.transpose()
        // }
        // hat_q_min = argmin_q;
        // if(hat_q_min == hat_qk){//converge
        //    return hat_q_min;
        // }else{
        //    hat_q = hat_qmin;
        //    k ++;
        // }
    }while(!(q==qk) && k < maxIterations);
    return q.hat;
}

/**
 * @brief do scan matching to get related motion from kfi to kfn
 * 
 * @param kfn : Current KeyFrame
 * @param kfi : KeyFrame before now
 * @return motion 
 */
motion LoopClosing::ScanMatching(KeyFrame* kfn, KeyFrame* kfi){
//对两full scan做扫描匹配得到相对运动及其不确定性估计
    motion d;
    std::cout<<"start scan matching, new kf is:"<<std::endl;
    // kfn->Print();
    std::cout<<"old kfi is:"<<std::endl;
    // kfi->Print();
    //获得初始位移估计hat_q
    motion q = pose(kfi->GetPose().hat,kfi->GetPose().P).tail2tail(pose(kfn->GetPose().hat,kfn
    ->GetPose().P));
    //获得初始位移估计不确定性P_q
    // Eigen::Matrix<double, 3, 3> P_q;
    // Eigen::Matrix<double, 3, 3> H_k;
    // Eigen::Matrix<double, 3, 3> P_k;
    // P_q = H_k * P_k * H_k.transpose();
    std::cout<<"start modpIC with init_q "<<q.hat<<std::endl;
    d.hat = modpIC(kfn->GetSonarFullScan(),kfi->GetSonarFullScan(),q);
    d.P = 0.1*Eigen::Matrix3d::Identity(3,3);//TODO;
    return d;
}


} // namespace CS_SLAM
