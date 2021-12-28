#ifndef SONARDRAWER_H
#define SONARDRAWER_H


namespace CS_SLAM
{

class SonarDrawer{
public:
    SonarDrawer(LocalMap* pMap, const string &strSettingPath);
    viod DrawSonar();
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    LocalMap* mpMap;
};

}

#endif //SONARDRAWER_H