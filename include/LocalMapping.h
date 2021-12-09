#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H


namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class LocalMapping{
public:
    LocalMapping();
    ~LocalMapping();
    void SetLocalMapper(LocalMapping* localMapper);
    void SetScanFormer(ScanFormer* scanFormer);
    void Run();
    void UseMono(meas.raw_measurements_,dt,paramMono);

private:
    ScanFormer* mpScanFormer;
    LocalMapping* mpLocalMapper;
};

}

#endif // LOCALMAPPING_H