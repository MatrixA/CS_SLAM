#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <opencv2/opencv.hpp>
#include "ScanFormer.h"

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class LocalMapping{
public:
    LocalMapping();
    ~LocalMapping();
    void SetLocalMapper(LocalMapping* localMapper);
    void SetScanFormer(ScanFormer* scanFormer);
    void Run();
    void UseMono(cv::Mat raw_measurements_,double dt);

private:
    ScanFormer* mpScanFormer;
    LocalMapping* mpLocalMapper;
};

}

#endif // LOCALMAPPING_H