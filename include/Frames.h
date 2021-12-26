#ifndef FRAMES_H
#define FRAMES_H


namespace CS_SLAM
{

class Frames{
public:
    Frames();
    void add(KeyFrame* pKF);
    void erase(KeyFrame* pKF);

    void clear();
    void clearMap(Map* pMap);

private:
    std::mutex mMutex;
};

}

#endif //FRAMES_H