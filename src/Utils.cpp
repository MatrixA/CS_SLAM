#include "Utils.h"

namespace CS_SLAM
{
const int Utils::daysPerMonth[]={0,31,59,90,120,151,181,212,243,273,304,334,365};
double Utils::MahDistance(const Eigen::VectorXd &ri, const Eigen::MatrixXd &Sigma, const Eigen::VectorXd &ni){
/*
输入：向量ri,协方差矩阵，向量ni
输出：ri和ni的马氏距离
*/
    return sqrt((ri - ni).transpose() * Sigma.inverse() * (ri - ni));
}

Eigen::Vector2d Utils::Oplus(const Eigen::Vector3d &q,const Eigen::Vector2d &n){
    /*
        输入:不确定点p和不确定位移q
        输出:位移后的不确定点np
    */
    Eigen::Vector2d np(n(0)*cos(q(2))-n(1)*sin(q(2))+q(0), n(0)*sin(q(2))+n(1)*cos(q(2))+q(1));
    return np;
}

Eigen::Vector3d Utils::Odot(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
    return Eigen::Vector3d(a(0)+b(0),a(1)+b(1),b(2));
}

/**
 * @brief transform unix timestamp/ns to datetime
 * 
 * @param timestamp - unix timestamp/ns
 * @return std::string - datetime string
 */
std::string Utils::TimeStamp2TimeString(unsigned long long timestamp){
    int totsec = timestamp/1e9; int sec = totsec % 60;
    int totmin = totsec/60; int minute = totmin % 60;
    int tothours = totmin/60; int hour = tothours % 24;
    int totday = tothours/24;
    int year = totday/365+1970;
    int dayleft = totday - ((year-1970)*365 + (year-1970)/4);
    int month = dayleft/30+1;
    int day = dayleft - daysPerMonth[month-1];//will be a BUG in the future!
    for(int i=1;i<=12;i++){
        if(daysPerMonth[i]>=day){
            day = day - daysPerMonth[i-1];
            break;
        }
    }
    return std::to_string(year)+"."+std::to_string(month)+"."+std::to_string(day)+" "
            +std::to_string(hour)+":"+std::to_string(minute)+":"+std::to_string(sec);
}



}