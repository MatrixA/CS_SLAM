#ifndef MAPDRAWER_H
#define MAPDRAWER_H


namespace CS_SLAM
{

class Drawer{
public:
    Drawer(Atlas* pAtlas, const string &strSettingPath);

    Atlas* mpAtlas;

    void DrawMapPoints();
    viod DrawSonar();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
};

}

#endif