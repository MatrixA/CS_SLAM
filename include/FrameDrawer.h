#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H


namespace CS_SLAM
{

class FrameDrawer{
public:
    FrameDrawer(Map* pMap, const string &strSettingPath);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    Map* mpMap;
};

}

#endif //FRAMEDRAWER_H