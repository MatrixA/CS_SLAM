#include "System.h"

namespace CS_SLAM
{

System::System(YAML::Node SensorConfig){
    //viewer_thread_ = std::thread(std::bind(&Viewer:))
    //viewer_thread_ = std::thread(std::bind(&System::Plot, this));
    // mpAtlas = new Atlas();
    mSensorConfig = SensorConfig;
    mpMap = new LocalMap();
    mpFramesDatabase = new Frames();
    mpViewer = new Viewer(mpMap, mpFramesDatabase);
    mptViewer = new std::thread(&Viewer::ThreadLoop,mpViewer);

    mpEKF = new EKF();
    mpMSCKF = new MSCKF(mpFramesDatabase);
    mpScanFormer = new ScanFormer(mpEKF);
    mpLoopClosing = new LoopClosing(mpFramesDatabase);
    mpLocalMapper = new LocalMapper(mpEKF, mpMSCKF, mpFramesDatabase);
}

System::~System(){
    
}

void System::Initialize(){

    mbInit = true;
}
/**
 * @brief save Trajectory from MSCKF using filename
 *        format : id x y z
 * @param filename 
 */
void System::SaveTrajectory(const std::string &filename){
    std::ofstream outfile;
    outfile.open(filename);
    if(!outfile.is_open()){
        std::cout<<"open file fail"<<std::endl;
        return;
    }
//MSCKF
    Eigen::VectorXd poses = mpMSCKF->GetX();
//DR
    // Eigen::VectorXd poses = mpScanFormer->getx_ss();
    int tot = poses.size()/3;
    std::cout<<"total "<<tot<<" poses. "<<std::endl;
    for(int i = 0; i<poses.size()/3;i++){
        KeyFrame* tmpKF = mpFramesDatabase->GetKeyFrameByID(i);
        
        outfile << std::setprecision(12)<<(double)tmpKF->GetTimeStamp()/1e9 << " "<< poses(3*i) <<" "<< poses(3*i+1) <<" "<< poses(3*i+2) << std::endl;
        // outfile << i+1 << " "<< poses(3*i) <<" "<< poses(3*i+1) <<" "<< poses(3*i+2) << std::endl;
    }
    outfile.close();
    std::cout<<"write OK"<<std::endl;
}

/**
 * @brief save Trajectory from Database
 *        format : id x y z
 * @param filename 
 */
void System::SaveTrajectoryFromDatabase(const std::string &filename){
    std::ofstream outfile;
    outfile.open(filename);
    if(!outfile.is_open()){
        std::cout<<"open file fail"<<std::endl;
        return;
    }
    for(int i=0;i < mpFramesDatabase->Size();i++){
        KeyFrame* tmpKF = mpFramesDatabase->GetKeyFrameByID(i);
        Eigen::Vector3d tmp = (tmpKF->GetPose()).hat;
        // outfile<< i+1 << " " << tmp(0)<<" "<< tmp(1) <<" "<< tmp(2)<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<std::endl;
        //tum: time, x,y,z,qx,qy,qz,qw;
        outfile<<std::setprecision(12)<<(double)tmpKF->GetTimeStamp()/1e9 << " " << tmp(0)<<" "<< tmp(1) <<" "<< tmp(2)<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<std::endl;
    }
    std::cout<<"write OK"<<std::endl;
    outfile.close();
}

/**
 * @brief using AHRS measurement to update EKF in ScanFormer 
 * 
 * @param meas - AHRS measurement
 */
void System::TrackAHRS(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        std::cout<<"first is "<<meas.sensor_type_<<std::endl;
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

/**
 * @brief using DVL measurement to update EKF in ScanFormer
 * 
 * @param meas - DVL measurement
 */
void System::TrackDVL(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        std::cout<<"first is "<<meas.sensor_type_<<std::endl;
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

/**
 * @brief using DS measurement to update EKF in ScanFormer
 * 
 * @param meas - DS measurement
 */
void System::TrackDS(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        std::cout<<"first is "<<meas.sensor_type_<<std::endl;
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

/**
 * @brief using sonar measurement to update EKF in Scan Former and form full scan
 * 
 * @param meas - sonar data
 */
void System::TrackSonar(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        std::cout<<"first is "<<meas.sensor_type_<<std::endl;
        // mpFramesDatabase->add(KeyFrame());
        // mpViewer->RefreshCurrentFrame(timestamp_now);
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
        if(!mpMSCKF->IsInitialized()){
            std::cout<<"Initialize MSCKF:"<<(mpScanFormer->GetFullMotion().hat)<<std::endl;
            mpMSCKF->Initialize(mpScanFormer->GetFullMotion());
        }else{
            // std::cout<<"AddPose"<< mpScanFormer->GetFullMotion().GetPose().hat<< std::endl;
            // std::cout<<"predict "<<std::endl;
            mpMSCKF->Prediction(mpScanFormer->GetFullMotion());
            KeyFrame* curP=new KeyFrame(mpMSCKF->GetCurrentPose());
            curP->SetTimeStamp(meas.timestamp_);
            // std::cout<<"fs size:"<<(mpScanFormer->GetFullScan()).size()<<std::endl;
            curP->SetSonarFullScan(mpScanFormer->GetFullScan());
            // curP->SetSonarMeasurements(mpScanFormer->GetScan());
            mpFramesDatabase->add(*curP);
            // std::cout<<"added in Sonar:"<<std::endl;
            // curP->Print();
            // KeyFrame* curP = mpFramesDatabase->GetCurrentKeyFrame();
            mpViewer->RefreshCurrentFrame(timestamp_now);
            const std::vector<int>& alternative = mpFramesDatabase->GetCurrentOverlaps(1);
            for(int i=0; i<alternative.size(); i++){
                KeyFrame* agoP = mpFramesDatabase-> GetKeyFrameByID(alternative[i]);
                motion estimate = mpLoopClosing->ScanMatching(curP,agoP);
                std::cout<<"detected "<<alternative[i]<<std::endl;
                //FrameDatabase里的第i个pose是MSCKF里的第n-1-i个pose】
                mpMSCKF->Update(mpFramesDatabase->Size()-1-alternative[i],0,estimate);
            }
        }
        // mpMSCKF.Update(mpScanFormer->GetFullMotion());
        mpScanFormer->Reset();
    }
//         if(scnt == FSCAN_SIZE){
//             full_scan_list.push_back(mpScanFormer->undistort());
//             mpScanFormer->reset();
//             scnt = 0;
//         }
    timestamp_now = meas.timestamp_;
    return ;
}

void System::TrackMono(MeasurementPackage meas){
    if(timestamp_now == 0){
        timestamp_now = meas.timestamp_;
        std::cout<<"first is "<<meas.sensor_type_<<std::endl;
        return ;
    }
    double dt = (double)(meas.timestamp_ - timestamp_now)/1e9;
    // std::cout<<"dt:"<<dt<<std::endl;
    if(dt < 0){
        std::cerr << "ERROR: datas not sequential." << std::endl;
        exit(-1);
    }
    // std::cout<<"--Start mpScanFormer->UseDS"<<std::endl;
    // mpMSCKF->Prediction(mpScanFormer->GetFullMotion());
    KeyFrame* curP=new KeyFrame(mpMSCKF->GetCurrentPose());
    curP->SetTimeStamp(meas.timestamp_);
    // std::cout<<"try "<<meas.filename<<std::endl;
    curP->LoadCameraImg(meas.filename);
    //至少第二帧图像时才刷新
    mpLocalMapper->UseMono(curP, dt, 50);
    //     mpFramesDatabase->add(*curP, 1);
    // }
    // mpViewer->add(*curP, 1);
    mpViewer->RefreshCurrentFrame(timestamp_now);
    
    // std::cout<<"Add curP OK"<<std::endl;
    timestamp_now = meas.timestamp_;
    return ;
}

void System::SetUp(){
    // pangolin::CreateWindowAndBind(window_name, 640,480);
    // glEnable(GL_DEPTH_TEST);
    // pangolin::GetBoundWindow()->RemoveCurrent();
    mbSetUp = true;
    return ;
}

bool System::isInitialized(){
    return mbInit;
}



// void System::Plot(){
//     pangolin::BindToContext(window_name);
//     glEnable(GL_DEPTH_TEST);
//     while( !pangolin::ShouldQuit() ){
//         Eigen::VectorXd all_pose=mpMSCKF.GetX();
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
//     }
//     pangolin::GetBoundWindow()->RemoveCurrent();
// }

//绘制轨迹
// void System::PlotTrajectory(){
//     if(!mpScanFormer->IsFull()){
//         std::cout<<"not Full no Plot"<<std::endl;
//         return ;
//     }
//     if(!mbSetUp){
//         SetUp();
//     }else{
//         std::cout<<"Full Start Plot"<<std::endl;
//         //std::thread render_loop(Plot);
//         //std::thread render_loop;
//         //render_loop = std::thread(System::Plot);
//         //render_loop.detach();
//         //std::thread(std::bind(&Viewer::ThreadLoop, this)
//         //render_loop.join();
//         std::cout<<"Full End Plot"<<std::endl; 
//         mpScanFormer->Reset();
//     }
//     return ;//
// }


// std::vector<std::vector<point> > all_scan;

// std::vector<RandomVector> all_scan_pose;













}//namespace CS_SLAM