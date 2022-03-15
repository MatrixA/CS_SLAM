#ifndef LOCALMAPPER_H
#define LOCALMAPPER_H


#include "ScanFormer.h"
#include "Frames.h"
#include "KeyFrame.h"
#include <opencv2/opencv.hpp>
#include "Converter.h"
#include "Utils.h"
#include "MSCKF.h"

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class LocalMapper{
public:
    LocalMapper();
    LocalMapper(EKF* mpEKF_, MSCKF* mpMSCKF_, Frames* mpFramesDatabase_);
    ~LocalMapper();
    // void SetLocalMapper(LocalMapping* localMapper);
    // void SetScanFormer(ScanFormer* scanFormer);
    bool Track(KeyFrame* lastKF, KeyFrame* nowKF, cv::Mat& R, cv::Mat& t, int feat_thresh);
    bool UseMono(KeyFrame* nwKeyFrame, double dt, int featThresh);

private:
    // ScanFormer* mpScanFormer;
    Frames* mpFramesDatabase;
    EKF* mpEKF;
    MSCKF* mpMSCKF;
    cv::Mat cameraMatrix=(cv::Mat_<double>(3,3)<< 707.0912,0,601.8873,0,707.0912,183.1104,0,0,1);

};

}

#endif // LOCALMAPPER_H