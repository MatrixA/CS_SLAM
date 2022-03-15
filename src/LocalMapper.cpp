#include "LocalMapper.h"

namespace CS_SLAM
{
    LocalMapper::LocalMapper(){}
    LocalMapper::LocalMapper(EKF* mpEKF_, MSCKF* mpMSCKF_, Frames* mpFramesDatabase_):mpEKF(mpEKF_), mpMSCKF(mpMSCKF_),mpFramesDatabase(mpFramesDatabase_){}
    LocalMapper::~LocalMapper(){}
    // void LocalMapper::AddKeyFrame(KeyFrame* pKF){
    //     keyFrames.push_back(pKF);
    // }
    // void LocalMapper::AddMapPoint(MapPoint* pMP){
    //     mapPoints.push_back(pMP);
    // }

    // int CalcMotion(std::vector<cv::Point2f>& pts1,std::vector<cv::Point2f>& pts2, cv::Mat& R, cv::Mat& t){
    //     // int ptCount = pts1.size();
    // //     cv::Mat *p1 = cv::cvCreateMat(ptCount, 2, CV_32F);
    // //     cv::Mat p2(ptCount, 2, CV_32F);
    
    // // // 把Keypoint转换为Mat
    // //     cv::Point2f pt;
    // //     for (int i=0; i<ptCount; i++)
    // //     {
    // //         pt = pts1[i];
    // //         p1.at<float>(i, 0) = pt.x;
    // //         p1.at<float>(i, 1) = pt.y;
        
    // //         pt = pts2[i];
    // //         p2.at<float>(i, 0) = pt.x;
    // //         p2.at<float>(i, 1) = pt.y;
    // //     }
    //     std::cout<<pts1.size()<<"|"<<pts2.size()<<std::endl;
    //     //cv::Mat fundamental_matrix;
    //     //fundamental_matrix = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 3, 0.99);
    //     //std::cout<<"fundamental_matrix is"<<std::endl<<fundamental_matrix<<std::endl;
    //     //cv::Mat cameraMatrix=(cv::Mat_<double>(3,3)<< 405.6385,0,189.9054,0,405.5883,139.9150,0,0,1);
    //     cv::Mat essential_matrix;
    //     essential_matrix = cv::findEssentialMat(pts1,pts2,cameraMatrix);
    //     std::cout<<"have essential matrix"<<std::endl;
    //     cv::recoverPose(essential_matrix, pts1, pts2, cameraMatrix,R,t);
    //     cv::Vec3f euler= rotationMatrixToEulerAngles(R);
    //     std::cout<<"angle:"<<euler<<std::endl;
    //     return 1;
    // }
/**
 * @brief track optical flow using img1 and img2
 * 
 * @param img1 
 * @param img2 
 * @param R 
 * @param t 
 * @param feat_thresh 
 * @return int 
 */
    bool LocalMapper::Track(KeyFrame* lastKF, KeyFrame* nowKF, cv::Mat& R, cv::Mat& t, int feat_thresh){
        cv::Mat img1 = lastKF->GetCameraImage();
        cv::Mat img2 = nowKF->GetCameraImage();
        int num_good_pts=0;
        std::vector<cv::Point2f> pts1, pts2;
        // cv::Mat desc1,desc2;
        // cv::Mat dest1;
        // cv::Ptr<cv::ORB> detector = cv::ORB::create();
        // detector->detect(img1,pts1,cv::Mat());
        // cv::drawKeypoints(img1, pts1, dest1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("output", dest1);
        std::vector<uchar> status;
        std::vector<float> err;
        cv::Mat old_frame,cur_frame;
        cv::cvtColor(img1, old_frame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(old_frame, pts1, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
        cv::cvtColor(img2, cur_frame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(cur_frame, pts2, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
        if(pts1.size()==0 || pts2.size()==0)return false;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        cv::calcOpticalFlowPyrLK(old_frame, cur_frame, pts1, pts2, status, err, cv::Size(15,15), 2, criteria);
        cv::Mat mask = cv::Mat::zeros(img1.size(), img1.type());
        std::vector<cv::Point2f> good_new;
        for(uint i = 0; i < pts1.size(); i++)
        {
            // std::cout<<"good "<<i<<std::endl;
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(pts2[i]);
                // draw the tracks
                line(mask,pts2[i], pts1[i], cv::Scalar(255,239,213), 5);
                circle(img2, pts2[i], 1, cv::Scalar(0.01+0.03*i,0.01+0.03*i,0.01+0.03*i), -1);
                num_good_pts++;
            }
        }
        // if(num_good_pts>= feat_thresh)std::cout<<"should be"<<std::endl;
        // std::cout<<"only "<<num_good_pts<<std::endl;
        if(num_good_pts< feat_thresh)return false;
        // cv::Mat img;
        // cv::add(img2, mask, img);
        // cv::imshow("input", img);
        //pts1和pts2为img1和img2的对应匹配点
        //calculate motion
        cv::Mat essential_matrix = cv::findEssentialMat(pts1,pts2,cameraMatrix);
        cv::Mat Rtmp,ttmp;
        cv::recoverPose(essential_matrix, pts1, pts2, cameraMatrix,Rtmp,ttmp);

        //转化为DVL之间的运动
        t = (cv::Mat_<double>(3,1)<<0.02*Rtmp.at<double>(0,1)-0.26*Rtmp.at<double>(0,0)+ttmp.at<double>(0,0)+0.26,
                                    0.02*Rtmp.at<double>(2,1)-0.26*Rtmp.at<double>(2,0)+ttmp.at<double>(2,0),
                                    0.02*Rtmp.at<double>(1,1)-0.26*Rtmp.at<double>(1,0)+ttmp.at<double>(1,0)-0.02);
        // t.at<double>(0,0)=0.02*Rtmp.at<double>(0,1)-0.26*Rtmp.at<double>(0,0)+ttmp.at<double>(0,0)+0.26;
        // t.at<double>(1,0)=0.02*Rtmp.at<double>(2,1)-0.26*Rtmp.at<double>(2,0)+ttmp.at<double>(2,0);
        // t.at<double>(2,0)=0.02*Rtmp.at<double>(1,1)-0.26*Rtmp.at<double>(1,0)+ttmp.at<double>(1,0)-0.02;
        cv::Mat assis=(cv::Mat_<double>(3,3)<< 0,-1,0,1,0,0,0,0,1);
        R = assis*Rtmp*assis;
        // Toc<< 1,0,0,0.26,
        //       0,0,1,0,
        //       0,1,0,-0.02,
        //       0,0,0,1;
        
        // cv::Vec3f euler= Converter::RotationMatrixToEulerAngles(R);
        // std::cout<<R<<std::endl;
        // std::cout<<t<<std::endl;
        return true;
    }

/**
 * @brief 使用相机图像更新位姿
 * 
 * @param nwKeyFrame 
 * @param featThresh 
 * @return true - enough features
 * @return false - not enough features
 */
    bool LocalMapper::UseMono(KeyFrame* nwKeyFrame, double dt, int featThresh){
        mpEKF->prediction(dt);
        nwKeyFrame->Transform(mpEKF->GetPose());
        // std::cout<<"first cam ASEKF"<<mpEKF->GetPose().hat<<" and "<<mpMSCKF->GetCurrentPose().hat<<std::endl;
        //如果framedatabase里没有相机帧，就插入第一个相机帧并返回false表示未计算相机运动
        if(!mpFramesDatabase->IsInitiliedCam()){
            mpFramesDatabase->add(*nwKeyFrame,1);
            // std::cout<<"pushed first cam in FramesDatabase1:";
            // nwKeyFrame->Print();
            return false;
        }
        KeyFrame* lastKeyFrame = mpFramesDatabase->GetLastCameraKeyFrame();
        // std::cout<<"lastCamera address:"<<lastKeyFrame<<std::endl;
        
        cv::Mat Ro,to;
        bool ret=Track(lastKeyFrame,nwKeyFrame,Ro,to,featThresh);
        // std::cout<<"can of "<<ret<<std::endl;
        if(ret){
            motion dMotion = lastKeyFrame->GetPose().tail2tail(nwKeyFrame->GetPose());
            // tranform camera motion to DVL motion
            // Converter::CameraToDVL(Ro,to,Rodvl,todvl);
            // std::cout<<"use optical"<<std::endl;
            Eigen::Vector3d oMotionVec;
            oMotionVec(0)=to.at<double>(0,0);
            oMotionVec(1)=to.at<double>(1,0);
            oMotionVec(2)=Converter::RotationMatrixToEulerAngles(Ro)[2];
            //光流估计结果
            motion oMotion(oMotionVec,0.01*Eigen::MatrixXd::Identity(3,3));
            motion fMotion = Converter::FusionMotions(dMotion, oMotion, 0.2);
            nwKeyFrame->SetPose((lastKeyFrame->GetPose()).compound(fMotion));
            mpFramesDatabase->add(*nwKeyFrame,1);
            // std::cout<<"pushed first cam in FramesDatabase2:";
            // nwKeyFrame->Print();
        }else{
            // std::cout<<"no optical flow is used"<<std::endl;
            mpFramesDatabase->add(*nwKeyFrame,1);
            // std::cout<<"pushed first cam in FramesDatabase3:";
            // nwKeyFrame->Print();
        }
        return ret;
    }


} // namespace CS_SLAM
