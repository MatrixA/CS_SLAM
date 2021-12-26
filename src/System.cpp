#include "System.h"

namespace CS_SLAM
{

System::System(YAML::Node SensorConfig){
    //viewer_thread_ = std::thread(std::bind(&Viewer:))
    //viewer_thread_ = std::thread(std::bind(&System::Plot, this));
    // mpAtlas = new Atlas();
    mSensorConfig = SensorConfig;
    mpMap = new Map();
    mpViewer = new Viewer(mpMap);
    mptViewer = new std::thread(&Viewer::ThreadLoop,mpViewer);
    mpASEKF = new ASEKF();
    mpScanFormer = new ScanFormer();
}

System::~System(){
    
}

void System::SaveTrajectory(const std::string &filename){
    std::ofstream outfile;
    outfile.open(filename);
    if(!outfile.is_open()){
        std::cout<<"open file fail"<<std::endl;
        return;
    }
//ASEKF
    Eigen::VectorXd poses = mpASEKF->GetX();
//DR
    // Eigen::VectorXd poses = mpScanFormer->getx_ss();
    int tot = poses.size()/3;
    std::cout<<"total "<<tot<<" poses. "<<std::endl;

    for(int i = 0; i<poses.size()/3;i++){
        outfile << i+1 << " "<< poses(3*i) <<" "<< poses(3*i+1) <<" "<< poses(3*i+2) << std::endl;
    }
    std::cout<<"write OK"<<std::endl;
    outfile.close();
}

void System::TrackAHRS(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        return ;
    }
    double dt = (double)(meas.timestamp_ - timestamp_now)/1e9;
    // std::cout<<"dt:"<<dt<<std::endl;
    if(dt < 0){
        std::cerr << "ERROR: datas not sequential." << std::endl;
        exit(-1);
    }

    // std::cout<<"--Start mpScanFormer->UseAHRS"<<std::endl;
    mpScanFormer->UseAHRS(meas.raw_measurements_,dt);
    // std::cout<<"--Start mpScanFormer->UseAHRS"<<std::endl;
    timestamp_now = meas.timestamp_;
    return ;
}

void System::TrackDVL(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        return ;
    }
    double dt = (double)(meas.timestamp_ - timestamp_now)/1e9;
    // std::cout<<"dt:"<<dt<<std::endl;
    if(dt < 0){
        std::cerr << "ERROR: datas not sequential." << std::endl;
        exit(-1);
    }
    // std::cout<<"--Start mpScanFormer->UseDVL"<<std::endl;
    mpScanFormer->UseDVL(meas.raw_measurements_,dt);
    // std::cout<<"--End mpScanFormer->UseDVL"<<std::endl;
    timestamp_now = meas.timestamp_;
    return ;
}

void System::TrackDS(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        return ;
    }    
    double dt = (double)(meas.timestamp_ - timestamp_now)/1e9;
    // std::cout<<"dt:"<<dt<<std::endl;
    if(dt < 0){
        std::cerr << "ERROR: datas not sequential." << std::endl;
        exit(-1);
    }
    // std::cout<<"--Start mpScanFormer->UseDS"<<std::endl;
    mpScanFormer->UseDS(meas.raw_measurements_,dt);
    // std::cout<<"--Over mpScanFormer->UseDS"<<std::endl;
    timestamp_now = meas.timestamp_;
    return ;
}


void System::TrackSonar(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
    }
    // std::cout<<"dt:";
    double dt = (double)(meas.timestamp_ - timestamp_now)/1e9;
    // std::cout<<dt<<std::endl;
    if(dt < 0){
        std::cerr << "ERROR: datas not sequential." << std::endl;
        exit(-1);
    }
    
    // std::cout<<"--Start mpScanFormer->UseSonar"<<std::endl;
    // std::cout<<"In TrackSonar raw:"<<meas.raw_measurements_.size()<<std::endl;
    mpScanFormer->UseSonar(meas.raw_measurements_,dt);
    // std::cout<<"--Over mpScanFormer->UseSonar"<<std::endl;
    scnt++;
    if(mpScanFormer->IsFull()){
        if(!mpASEKF->IsInitialized()){
            std::cout<<"Initialize ASEKF:"<<(mpScanFormer->GetFullMotion().hat)<<std::endl;
            mpASEKF->Initialize(mpScanFormer->GetFullMotion());
        }else{
            // std::cout<<"AddPose"<< mpScanFormer->GetFullMotion().GetPos()<< std::endl;
            // std::cout<<"predict "<<std::endl;
            mpASEKF->Prediction(mpScanFormer->GetFullMotion());
        }
        // mpASEKF.Update(mpScanFormer->GetFullMotion());
        mpScanFormer->Reset();
    }
//         if(scnt == FSCAN_SIZE){
//             full_scan_list.push_back(mpScanFormer->undistort());
//             mpScanFormer->reset();
//             motion = modpIC(full_scan);
//             ASEKF.update(motion);
//             scnt = 0;
//         }
    timestamp_now = meas.timestamp_;
    return ;
}

void System::SetUp(){
    pangolin::CreateWindowAndBind(window_name, 640,480);
    glEnable(GL_DEPTH_TEST);
    pangolin::GetBoundWindow()->RemoveCurrent();
    mbSetUp = true;
    return ;
}

// void System::Plot(){
//     pangolin::BindToContext(window_name);
//     glEnable(GL_DEPTH_TEST);
//     while( !pangolin::ShouldQuit() ){
//         Eigen::VectorXd all_pose=mpASEKF.GetX();
//         int pointNums =  all_pose.size()/3;
//         glPointSize(20.0);
//         glBegin(GL_POINTS);
//         for(int i = 0; i < pointNums; i++){
//             glColor3f(0,1,0);//修改颜色
//             std::cout<<"Plot "<<i<<std::endl;
//             Eigen::Vector3d here(all_pose.middleRows(3*i,3));
//             glVertex3f(here(0),here(1),0);//设置定点坐标
//         }
//         glEnd();
//         pangolin::FinishFrame();
//     }z
//     pangolin::GetBoundWindow()->RemoveCurrent();
// }

//绘制轨迹
void System::PlotTrajectory(){
    if(!mpScanFormer->IsFull()){
        std::cout<<"not Full no Plot"<<std::endl;
        return ;
    }
    if(!mbSetUp){
        SetUp();
    }else{
        std::cout<<"Full Start Plot"<<std::endl;
        //std::thread render_loop(Plot);
        //std::thread render_loop;
        //render_loop = std::thread(System::Plot);
        //render_loop.detach();
        //std::thread(std::bind(&Viewer::ThreadLoop, this)
        //render_loop.join();
        std::cout<<"Full End Plot"<<std::endl; 
        mpScanFormer->Reset();
    }
    return ;//
}


std::vector<std::vector<point> > all_scan;

std::vector<RandomVector> all_scan_pose;

std::vector<int> getOverlap(Eigen::Vector3d x_nk, int threshold=1){
/*
输入：当前扫描对应的位姿x_nk，距离阈值threshold
输出：所有相邻扫描对应的位姿id数组
*/
    std::vector<int> ans;
    for(int i = 0; i < all_scan_pose.size(); i++){
        double dis = (all_scan_pose[i].hat - x_nk).norm();
        if(dis < threshold){
            ans.push_back(i);
        }
    }
    return ans;
}



// struct MahResidual{
// public:
//     MahResidual(point sr,point sn):ai(sr),ci(sn){}
//     template <typename T>
//     bool operator()(const T* const q0,const T* const q1, const T* const q2, T* residual) const{
//         Eigen::Vector2d ei(ai.hat(0)-ci.hat(0)*cos(q2)+ci.hat(1)*sin(q2)-q0,
//                             ai.hat(1)-ci.hat(0)*sin(q2)-ci.hat(1)*cos(q2)-q1);
//         Eigen::MatrixXd J_q, J_c;
//         J_q << 1, 0, -ci.hat(0)*sin(q2)-ci.hat(1)*cos(q2),
//                 0, 1, ci.hat(0)*cos(q2)+ci.hat(1)*sin(q2);
//         J_c << cos(q2), -sin(q2),
//                 sin(q2), -cos(q2);
//         Eigen::MatrixXd P = J_q*J_q.transpose() + J_c*ci.P*J_c.transpose();
//         // MatrixXd P = /* rj.P + */ J_q*q.P*J_q.transpose() + J_c*ci.P*J_c.transpose();
//         residual[0] = ei.transpose()*P.inverse()*ei;
//         return true;
//     }
// private:
//     point ai;
//     point ci;
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
//             new ceres::AutoDiffCostFunction<MahResidual, 1, 1, 1>(
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