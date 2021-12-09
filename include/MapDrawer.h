#ifndef MAPDRAWER_H
#define MAPDRAWER_H


namespace CS_SLAM
{

class MapDrawer{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);
    void DrawMapPoints();
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    Map* mpMap;
};

}

#endif //MAPDRAWER_H