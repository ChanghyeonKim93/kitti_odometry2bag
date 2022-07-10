#ifndef _TOPIC_SIMULATOR_H_
#define _TOPIC_SIMULATOR_H_

#include <iostream>
#include <vector>
#include <string>

#include <fstream>
#include <sstream>

// ROS eigen
#include <Eigen/Dense>

// ROS cv_bridge
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// ROS messages
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/TimeReference.h>

#include <sensor_msgs/Image.h>


#include "geometry_library.h"

using namespace std;
class KITTIOdometry2Bag {
public:

    KITTIOdometry2Bag(ros::NodeHandle& nh, std::string dir_dataodometry, std::string data_num);
    ~KITTIOdometry2Bag();

    void loadDataset();

    void publishNextTopics(const ros::Time& time_current);

    //
    void startTopicsSimulation(const ros::Time& rost);
    void endTopicsSimulation();

    double getElapsedTime();
    void setIndexRange(int idx0, int idx1);
    void calcTimeRange();

    bool isEnd();

    int getDataLength();


private:

    string dir_dataodometry_;
    string data_num_;

    int n_data_;
   
    int idx_data_;

    int idx_data_start_;
    int idx_data_end_;

    double timestamp_start_;
    double timestamp_end_;
    
    double roswalltime_start_;
    double roswalltime_dt_;

    double time_range_[2];

    // Data odometry dir & dataset number
    std::string filedir_poses_;
    std::string filedir_gray_;

    // Stereo gray images & ground truth
    vector<double> timestamps_;

    // for images
    vector<string> filedir_imgs0_; // left
    vector<string> filedir_imgs1_; // right
    
    vector<cv::Mat> imgs0_; // left
    vector<cv::Mat> imgs1_; // right

    // for poses (Ground truth)
    vector<geometry_msgs::PoseStamped> poses_;

    // Publisher
    ros::NodeHandle nh_;
    ros::Publisher pub_img0_;
    ros::Publisher pub_img1_;
    ros::Publisher pub_gt_;
};

#endif