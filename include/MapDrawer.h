#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "KeyFrame.h"

namespace CS_SLAM
{

class MapDrawer{
public:
    MapDrawer(Map* pMap, const std::string &strSettingPath);
    void DrawMapPoints();
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    Map* mpMap;
};

}

#endif //MAPDRAWER_H