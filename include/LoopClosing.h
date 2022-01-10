#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "RandomVector.h"
#include "MapPoint.h"
#include "Frames.h"
#include "Utils.h"
#include "boost/math/distributions.hpp"
#include <ceres/ceres.h>

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class LoopClosing{
public:
    LoopClosing();
    LoopClosing(Frames* KeyFrameDatabase);
    ~LoopClosing();
    void SetTracker();
    void Run();
    motion ScanMatching(KeyFrame* kfn, KeyFrame* kfi);
private:
    // ScanFormer* mpScanFormer;
    Frames* mpKeyFrameDatabase;
};

}

#endif // LOOPCLOSING_H