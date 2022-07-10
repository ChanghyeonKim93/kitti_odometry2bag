#include <ros/ros.h>
#include <iostream>
#include <exception>
#include <time.h>
#include <string>
#include <sstream>

#include "kitti_odometry2bag.h"
#include "keyinput.h"

#include <signal.h>
// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

using namespace std;
int main(int argc, char **argv) {
    signal(SIGINT, signal_callback_handler);
    ros::init(argc, argv, "kitti_odometry2bag");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("kitti_odometry2bag node starts.\n");
    
    string dir_dataodometry = "/home/kch/Documents/kitti";
    string data_num = "00";
    std::string dir;
    // ros::param::get("~directory", dir);

    std::string user_manual;
    std::stringstream ss;    
    ss  << "\n==============================================\n|" 
        << "  Select..." 
        << "\n|      [p]  publish topics in real-time."
        << "\n| Select an input: \n";
        user_manual = ss.str();
        cout << user_manual;
        ss.clear();
        ss.flush();

        ss << "\n |\n L\n";

    try{
        std::shared_ptr<KITTIOdometry2Bag> kitti_loader = nullptr;

        kitti_loader = std::make_shared<KITTIOdometry2Bag>(nh, dir_dataodometry, data_num);

        int idx_range[2] = {0,4539};
        kitti_loader->setIndexRange(idx_range[0], idx_range[1]);
        kitti_loader->calcTimeRange();
        
        while(ros::ok()){
            ros::spinOnce();
            int c = getch();
            if(c == 'p')
            {
                std::cout << "\n\n   publish topics....\n";
                ros::Time rostime_start = ros::Time::now();
                kitti_loader->startTopicsSimulation(rostime_start);

                while(1)
                {
                    ros::spinOnce();
                    kitti_loader->publishNextTopics(ros::Time::now());
                    if(kitti_loader->isEnd() || getch() == 'p'){
                        // reset publisher
                        std::cout << " done\n";
                        break;
                    }
                }
                std::cout << user_manual;
            }
        }
    }
    catch(std::exception& e) {
        std::cout << "\n[Exception]: " << e.what();
    }
    
    ROS_INFO_STREAM("TERMINATE: \"kitti_odometry2bag node\".\n");
    return -1;
}