#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H


namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class LoopClosing{
public:
    LoopClosing();
    ~LoopClosing();
    void SetTracker();
    void Run();

private:
    ScanFormer* mpScanFormer;

};

}

#endif // LOOPCLOSING_H