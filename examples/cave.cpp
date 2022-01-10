#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "System.h"
#include "MeasurementPackage.h"


void LoadImages(const std::string &strSequence, std::vector<std::string> &vstrImageFilenames,
                std::vector<double> &vTimestamps);

void Import(const std::string& filename, std::vector<CS_SLAM::MeasurementPackage> &data, std::vector<int> &data_ind, enum CS_SLAM::MeasurementPackage::SensorType sensor, bool tags){
    std::ifstream infile;
    infile.open(filename, std::ios::in);
    std::cout<<"Reading"<<std::endl;
    if(!infile.is_open()){
        std::cout<<"open file fail"<<std::endl;
        return;
    }
    std::string line;
    if(tags)getline(infile,line); // 第一行为标签
    while(getline(infile, line)){
        std::istringstream iss(line);
        std::string lineelems;
        CS_SLAM::MeasurementPackage meas_package(data_ind.size()-1);
        int cnt = 0, indcnt = 0;
        while(getline(iss,lineelems,',')){
            // std::cout<<":"<<lineelems.c_str();
            if(indcnt< data_ind.size() && cnt == data_ind[indcnt]){
                // std::cout<<"get "<<lineelems.c_str();
                if(indcnt == 0){
                    meas_package.timestamp_=atoll(lineelems.c_str());
                    indcnt++;
                }else{
                    //有一个时间戳不在raw_measurements_里
                    meas_package.raw_measurements_(indcnt-1) = atof(lineelems.c_str());
                    indcnt++;
                }
                // std::cout<< "got" << atof(lineelems.c_str())<<std::endl;
            }
            cnt++;
        }
        //std::cout<<"here"<<std::endl;
        meas_package.sensor_type_ = sensor;
        data.push_back(meas_package);
        //std::cout<<"here1"<<std::endl;
    }
    std::cout<<"tot:"<<data.size()<<std::endl;
    // std::cout<<data[0].raw_measurements_<<std::endl;
    return ;
}

// YAML::Node ConfigSensor(const std::string DB_CONF){
//     std::cout<<DB_CONF<<std::endl;
//     YAML::Node conf = YAML::LoadFile(DB_CONF);
//     // sonarParam.resize(3);
//     // sonarParam(0)=conf["Sonar"]["angle_step"].as<double>();
//     // std::cout<<sonarParam(0)<<std::endl;
//     return conf;
// }

void InputDataset(char **argv,std::vector<CS_SLAM::MeasurementPackage> &data, YAML::Node conf){
    std::ifstream infile;
    //数据读入模块
    std::string datafile_DVL = std::string(argv[1]) + "dvl_linkquest.txt";
    std::vector<int> datacols_DVL = {2,23,24,25};
    std::string datafile_AHRS = std::string(argv[1]) + "imu_adis.txt";
    std::vector<int> datacols_AHRS = {2,5};
    std::string datafile_Sonar = std::string(argv[1]) + "sonar_micron.txt";
    std::vector<int> datacols_Sonar(398);
    datacols_Sonar[0]=2;
    int itmp=7;
    std::generate(datacols_Sonar.begin()+1,datacols_Sonar.end(),[&itmp](){return itmp++;});
    std::string datafile_DS = std::string(argv[1]) + "depth_sensor.txt";
    std::vector<int> datacols_DS = {2,3};

    // for(int i=0;i<datacols_Sonar.size();i++)std::cout<<datacols_Sonar[i]<<":";
    std::vector<CS_SLAM::MeasurementPackage> data_DVL, data_AHRS, data_Sonar, data_DS;

    Eigen::VectorXd sonarParam, DVLParam, AHRSParam, DSParam;
    // ConfigSensor(string(argv[1]) + "SensorsConfiguration.yaml",sonarParam);
    //Sensor configuration
    const std::string DB_CONF=std::string(argv[2]);
    // conf = YAML::LoadFile(DB_CONF);
    
    Import(datafile_Sonar,data_Sonar,datacols_Sonar,CS_SLAM::MeasurementPackage::SONAR,true);
    Import(datafile_DVL,data_DVL,datacols_DVL,CS_SLAM::MeasurementPackage::DVL,true);
    Import(datafile_AHRS,data_AHRS,datacols_AHRS,CS_SLAM::MeasurementPackage::AHRS,true);
    Import(datafile_DS,data_DS,datacols_DS,CS_SLAM::MeasurementPackage::DS,true);

    data.insert(data.end(),data_DVL.begin(),data_DVL.end());
    data.insert(data.end(),data_AHRS.begin(),data_AHRS.end());
    data.insert(data.end(),data_Sonar.begin(),data_Sonar.end());
    data.insert(data.end(),data_DS.begin(),data_DS.end());
    std::sort(data.begin(),data.end());
}

int main(int argc, char **argv){
    if(argc != 3){
        std::cerr<<std::endl<<"Usage: path_to_settings path_to_sequence" << std::endl;
        return 1;
    }
//     vector<string> vstrImageFilenames;
//     vector<double> vTimestamps;
//     LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
//     int nImages = vstrImageFilenames.size();

//     unsigned int pd=0, pa=0, ps=0, scnt=0;
//     long long last_timestamp_ = 0;a
    std::vector<CS_SLAM::MeasurementPackage> dataSequence;
    YAML::Node sensorConfig;
    InputDataset(argv, dataSequence, sensorConfig);

    CS_SLAM::System SLAM(sensorConfig);
    // for(int i=0;i<3;i++){
    //     std::cout<<data[i].sensor_type_<<": "<<data[i].timestamp_<<std::endl;
    //     //std::cout<<data[i].raw_measurements_<<std::endl;
    // }
    int scnt=0;//统计声纳帧以形成fullscan触发不同的滤波器

//     scanFormer.reset();
//     ASEKF.reset();
    // std::cout<<dataSequence[1].sensor_type_<<std::endl;
    for(int i=0;i< dataSequence.size();i++){
//         //DVL或AHRS来了，就更新状态。Sonara来了，就用预测来估计状态。
        switch(dataSequence[i].sensor_type_){
            case CS_SLAM::MeasurementPackage::SONAR:
                // std::cout<<"Sonar Start"<<std::endl;
                SLAM.TrackSonar(dataSequence[i]);
                // std::cout<<"Sonar End"<<std::endl;
                break;
            case CS_SLAM::MeasurementPackage::DVL:
                // std::cout<<"DVL Start"<<std::endl;
                SLAM.TrackDVL(dataSequence[i]);
                // std::cout<<"DVL End"<<std::endl;
                break;
            case CS_SLAM::MeasurementPackage::AHRS:
                // std::cout<<"AHRS Start"<<std::endl;
                SLAM.TrackAHRS(dataSequence[i]);
                // std::cout<<"AHRS End"<<std::endl;
                break;
            case CS_SLAM::MeasurementPackage::DS:
                // std::cout<<"DS Start"<<std::endl;
                SLAM.TrackDS(dataSequence[i]);
                // std::cout<<"DS End"<<std::endl;
                break;
        }
//         // if(data[i].sensor_type_==CS_SLAM::MeasurementPackage::SONAR){
//         //     std::cout<<"Sonar Start"<<std::endl;
//         //     SLAM.TrackSonar(data[i],sonarParam);
//         //     std::cout<<"Sonar End"<<std::endl;
//         // }else if(data[i].sensor_type_==CS_SLAM::MeasurementPackage::DVL){
//         //     std::cout<<"DVL Start"<<std::endl;
//         //     SLAM.TrackDVL(data[i],DVLParam);
//         //     std::cout<<"DVL End"<<std::endl;
//         // }else if(data[i].sensor_type_==CS_SLAM::MeasurementPackage::AHRS){
//         //     std::cout<<"AHRS Start"<<std::endl;
//         //     SLAM.TrackAHRS(data[i],AHRSParam);
//         //     std::cout<<"AHRS End"<<std::endl;
//         // }else if(data[i].sensor_type_==CS_SLAM::MeasurementPackage::DS){
//         //     std::cout<<"DS Start"<<std::endl;
//         //     SLAM.TrackDS(data[i],DSParam);

//         //     std::cout<<"DS End"<<std::endl;
//         // }
//         SLAM.PlotTrajectory();
    }
    SLAM.SaveTrajectoryFromDatabase("../poses.txt");
    // SLAM.SaveTrajectory("../poses.txt");
    std::cout<<"nothing wrong"<<std::endl;
    usleep(5e6);
    return 0;
}


void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
        
            vTimestamps.push_back(t);
        }
    }

    std::string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
