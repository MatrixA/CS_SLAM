#include "MeasurementPackage.h"

namespace  CS_SLAM{

MeasurementPackage::MeasurementPackage(){};

MeasurementPackage::MeasurementPackage(int n){
    raw_measurements_.resize(n);
}

bool MeasurementPackage::operator< (const MeasurementPackage& meas) const{return this->timestamp_<meas.timestamp_;}

}


