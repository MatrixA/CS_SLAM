#ifndef FRAMES_H
#define FRAMES_H
#include "KeyFrame.h"
#include "LocalMap.h"
#include "RandomVector.h"
#include <mutex>
#include <vector>
#include <fstream>

namespace CS_SLAM
{

class Frames{
public:
    Frames();
    ~Frames();
    void add(KeyFrame pKF, int status=0);
    // void erase(KeyFrame* pKF);
    void erase(int ind);

    void clear();
    void clearMap(LocalMap* pMap);
    void AlterPose(int id, pose ps);

    int Size();
    KeyFrame* GetCurrentKeyFrame();
    KeyFrame* GetKeyFrameByID(int id, bool sonar/*=true*/);
    KeyFrame* GetLastCameraKeyFrame();

    bool IsInitiliedCam();
    bool HaveFrames();

    std::vector<int> GetOverlaps(KeyFrame kf, int threshold /*=1*/);
    std::vector<int> GetCurrentOverlaps(int threshold /*=1*/);
    
    void Init2DFromFile(std::string filename, bool haveObs);

private:
    std::mutex mMutex;
    std::vector<KeyFrame> KeyFrameDatabase;
    std::vector<int> IndOfSonarKeyFrames;
    KeyFrame* mpLastCameraKeyFrame;
};

}

#endif //FRAMES_H