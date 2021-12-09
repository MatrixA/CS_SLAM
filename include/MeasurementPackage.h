#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <vector>

/*
MeasurementPackage类定义
目的：获得timestamp_、raw_measurements_的数据项
*/
namespace CS_SLAM{

class MeasurementPackage {
public:
    MeasurementPackage();
    MeasurementPackage(int n);
    unsigned long long timestamp_;
    enum SensorType{
        CAMERA,
        SONAR,
        AHRS,
        DVL,
        DS
    } sensor_type_;
    bool operator< (const MeasurementPackage& meas)const ;
    Eigen::VectorXd raw_measurements_;
    cv::Mat raw_image;
};

}

#endif /* MEASUREMENT_PACKAGE_H */