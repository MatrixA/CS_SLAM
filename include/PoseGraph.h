#ifndef POSEGRAPH_H
#define POSEGRAPH_H


namespace CS_SLAM{
    class PoseGraph{
    public:
        PoseGraph(){}
        ~PoseGraph(){}
        void Initialize(Eigen::Vector3d x){
        /*
        输入：第一个可用的朝向测量
        输出（内含）：位姿图初始化为含有该位姿的简单图
        */
            if(!initialize_){
                all_scan_pose.push_back(x);
                initialize_ = true;
            }
            return ;
        }
        void Update(motion sm){
            Eigen::VectprXd y = z - H_ * ;
            return ;
        }

    private:
        bool initialize_ = false;
        vector<KeyFrame> 
    };
}

#endif