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
    void add(KeyFrame pKF);
    // void erase(KeyFrame* pKF);
    void erase(int ind);

    void clear();
    void clearMap(LocalMap* pMap);

    int Size();
    KeyFrame* GetCurrentKeyFrame();
    KeyFrame* GetKeyFrameByID(int id);

    std::vector<int> GetOverlaps(KeyFrame kf, int threshold /*=1*/);
    std::vector<int> GetCurrentOverlaps(int threshold /*=1*/);
    
    void Init2DFromFile(std::string filename, bool haveObs);

private:
    std::mutex mMutex;
    std::vector<KeyFrame> KeyFrameDatabase;
};

}

#endif //FRAMES_H