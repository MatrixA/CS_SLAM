#ifndef SONARDRAWER_H
#define SONARDRAWER_H


namespace CS_SLAM
{

class SonarDrawer{
public:
    SonarDrawer(Map* pMap, const string &strSettingPath);
    viod DrawSonar();
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    Map* mpMap;
};

}

#endif //SONARDRAWER_H