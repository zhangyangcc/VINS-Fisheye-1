#include<iostream>
#include <opencv2/opencv.hpp>
using namespace std;
#include "estimator/parameters.h"

int main()
{
    string config_file = "/home/zy/ros_ws/omni_swarm_ws/src/VINS-Fisheye-1/config/euroc/euroc_stereo_imu_config.yaml";
    
    // string IMAGE0_TOPIC_;
    // std::string COMP_IMAGE0_TOPIC;
    // cv::FileStorage fsSettings;
    // fsSettings.open(config_file.c_str(), cv::FileStorage::READ);
    // cv::Mat tmp;
    // fsSettings["compressed_image0_topic"] >> COMP_IMAGE0_TOPIC;
    // fsSettings["body_T_cam0"] >> tmp;
    // std::cout<<tmp<<std::endl;

    readParameters(config_file);
    std::cout<<RIC[0]<<std::endl;

    std::cout<<"heool"<<std::endl;
}