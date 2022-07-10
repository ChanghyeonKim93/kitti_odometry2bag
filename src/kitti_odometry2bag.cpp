#include "kitti_odometry2bag.h"

KITTIOdometry2Bag::KITTIOdometry2Bag(ros::NodeHandle& nh, std::string dir_dataodometry, std::string data_num) 
: nh_(nh), dir_dataodometry_(dir_dataodometry), data_num_(data_num)
{
    this->idx_data_ = 0;

    std::string topicname = "/kitti_odometry/left/image_raw";
    pub_img0_ = nh_.advertise<sensor_msgs::Image>(topicname, 1);

    topicname = "/kitti_odometry/right/image_raw";
    pub_img1_ = nh_.advertise<sensor_msgs::Image>(topicname, 1);

    topicname = "/kitti_odometry/groundtruth";
    pub_gt_   = nh_.advertise<geometry_msgs::PoseStamped>(topicname, 1);

    // load dataset
    this->loadDataset();

};

KITTIOdometry2Bag::~KITTIOdometry2Bag(){

};

void KITTIOdometry2Bag::loadDataset(){

    filedir_gray_  = dir_dataodometry_ + "/data_odometry_gray/dataset/sequences/" + data_num_  + "/";
    filedir_poses_ = dir_dataodometry_ + "/data_odometry_poses/dataset/poses/" + data_num_ + ".txt";

    // Image time.
    std::ifstream file_timestamp(filedir_gray_ + "times.txt");
    if(file_timestamp.is_open() == true){

        while(file_timestamp){
            std::string s;
            getline(file_timestamp, s);

            if(s.empty() ) break;

            ++n_data_;
            double t = stod(s);
            timestamps_.push_back(t);
        }
        file_timestamp.close();
    }
    else std::cout << "file open failed\n";

    std::cout << "n_data: " << n_data_ << std::endl;
  
    // Image file names
    for(int i = 0; i < n_data_; ++i){
        std::stringstream ss;
        ss << "image_0/" << std::setw(6) << std::setfill('0') << i << ".png";
        filedir_imgs0_.push_back(filedir_gray_+ss.str());

        ss.str("");
        ss << "image_1/" << std::setw(6) << std::setfill('0') << i << ".png";
        filedir_imgs1_.push_back(filedir_gray_+ss.str());
    }

    // Get images
    for(int i = 0; i < n_data_; ++i){
        cv::Mat img0,img1;
        img0 = cv::imread(filedir_imgs0_[i], cv::IMREAD_GRAYSCALE);
        img1 = cv::imread(filedir_imgs1_[i], cv::IMREAD_GRAYSCALE);

        imgs0_.push_back(img0);
        imgs1_.push_back(img1);
        std::cout << "get " << i << "-th images...\n";
    }

    // // Poses
    double mat[12];
    std::ifstream file_poses(filedir_poses_);
    if(file_poses.is_open() == true){
        while(file_poses){
            std::string s;
            getline(file_poses, s);
            
            if(s.empty() ) break;
            std::stringstream ss;
            ss << s;
            ss >> mat[0] >> mat[1] >> mat[2] >> mat[3] 
               >> mat[4] >> mat[5] >> mat[6] >> mat[7] 
               >> mat[8] >> mat[9] >> mat[10] >> mat[11];               

            geometry_msgs::PoseStamped msg;
            msg.pose.position.x = mat[3];
            msg.pose.position.y = mat[7];
            msg.pose.position.z = mat[11];

            Eigen::Matrix3d R;
            R << mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10];
            Eigen::Vector4d q = geometry::r2q(R);

            msg.pose.orientation.w = q(0);
            msg.pose.orientation.x = q(1);
            msg.pose.orientation.y = q(2);
            msg.pose.orientation.z = q(3);

            poses_.push_back(msg);
        }
        file_poses.close();
    }
    else std::cout << "file open failed\n";

    std::cout << "Data get done.\n";
};

void KITTIOdometry2Bag::publishNextTopics(const ros::Time& time_current) {
    ros::Time roswalltime_curr = time_current;
    roswalltime_dt_ = roswalltime_curr.toSec() - this->roswalltime_start_;
    if(idx_data_ < idx_data_end_ &&
        timestamps_[idx_data_] < roswalltime_dt_ + timestamp_start_){

        // publish images and lidars
        sensor_msgs::ImagePtr msg0 = 
        cv_bridge::CvImage(std_msgs::Header(), "mono8", imgs0_[idx_data_]).toImageMsg();
        msg0->header.frame_id = "cam";
        msg0->header.seq      = idx_data_;
        msg0->header.stamp    = roswalltime_curr;
        pub_img0_.publish(msg0);

        sensor_msgs::ImagePtr msg1 = 
        cv_bridge::CvImage(std_msgs::Header(), "mono8", imgs1_[idx_data_]).toImageMsg();
        msg1->header.frame_id = "cam";
        msg1->header.seq      = idx_data_;
        msg1->header.stamp    = roswalltime_curr;
        pub_img1_.publish(msg1);

        
        poses_[idx_data_].header.stamp    = roswalltime_curr;
        poses_[idx_data_].header.frame_id = "map";
        poses_[idx_data_].header.seq      = idx_data_;
        pub_gt_.publish(poses_[idx_data_]);
        
        cout << idx_data_ << "-th images (left, right) & pose pub! : " << timestamps_[idx_data_] << "\n";
        ++idx_data_;
    }
};

void KITTIOdometry2Bag::startTopicsSimulation(const ros::Time& rost){
    this->idx_data_          = idx_data_start_;
    this->roswalltime_start_ = rost.toSec();
};

void KITTIOdometry2Bag::endTopicsSimulation(){
    this->idx_data_ = idx_data_start_;
};

double KITTIOdometry2Bag::getElapsedTime(){
    return this->roswalltime_dt_;
};

void KITTIOdometry2Bag::setIndexRange(int idx0, int idx1){
    float time_start = timestamps_[idx0];
    float time_end   = timestamps_[idx1];
    
    for(int i = 0; i < timestamps_.size(); ++i){
        float time_now = timestamps_[i];
        if(time_now > time_start){
            idx_data_start_ = i;
            break;
        }
    }
    for(int i = 0; i < timestamps_.size(); ++i){
        float time_now = timestamps_[i];
        if(time_now > time_end){
            idx_data_end_ = i;
            break;
        }
    }

    timestamp_start_ = timestamps_[idx_data_start_];
    timestamp_end_   = timestamps_[idx_data_end_];
};

void KITTIOdometry2Bag::calcTimeRange(){
    this->time_range_[0] = this->timestamp_start_;
    this->time_range_[1] = this->timestamp_end_;

    //mcutime_range_[0] -= 1.0; // offset for stability
    //mcutime_range_[1] += 1.0; // offset for stability
    std::cout << " time range : " << this->time_range_[0] << " ~ " << this->time_range_[1] << std::endl;
};

bool KITTIOdometry2Bag::isEnd(){
    return ( (this->timestamp_start_ + this->roswalltime_dt_) > this->timestamp_end_);
};
