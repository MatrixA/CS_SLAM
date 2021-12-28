#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "LocalMap.h"
#include "KeyFrame.h"

namespace CS_SLAM
{

class MapDrawer{
public:
    MapDrawer(LocalMap* pMap, const std::string &strSettingPath);
    void DrawMapPoints();
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    LocalMap* mpMap;
};

}

#endif //MAPDRAWER_H