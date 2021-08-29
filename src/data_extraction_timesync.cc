#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <gps_common/conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

using TimeSyncType = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::NavSatFix, sensor_msgs::Imu>;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::NavSatFix, sensor_msgs::Imu>;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>{
public:
    void CreateMsg(const boost::shared_ptr<M const> &msg){this->signalMessage(msg);}
};

struct PoseData{
    double seq;
    double position_x;
    double position_y;
    double position_z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};

void CsvWritePose(std::ofstream &ostream, const PoseData &data);
void PopulatePoseData(PoseData &data, uint32_t seq, const sensor_msgs::NavSatFix::ConstPtr nav_ptr, const sensor_msgs::Imu::ConstPtr imu_ptr);
void WriteSyncData(const sensor_msgs::Image::ConstPtr &depth_ptr,
                   const sensor_msgs::Image::ConstPtr &rgb_ptr,
                   const sensor_msgs::NavSatFix::ConstPtr &nav_ptr,
                   const sensor_msgs::Imu::ConstPtr &imu_ptr);

namespace{
    std::string ex_path;  // Path to extraction directory
    std::ofstream gt_file;

    bool stop_flag = false;
}

int main(int argc, char const *argv[]){
    
    // Desired structure of extraction directory
    // -- camvox_dataset (extration directory)
    // ----| color_img
    // ----| depth_img

    if(argc != 3){
        ROS_ERROR("Usage: rosrun camvox_data_extraction <path-to-rosbag> <path-to-extraction-dir>('/'not included)");
        return EXIT_FAILURE;
    }

    ros::Time::init();

    std::string bag_path(argv[1]);
    ex_path = std::string(argv[2]);

    // Setup output file stream
    boost::format fmt_gt("%s/groundtruth.csv");
    gt_file = std::ofstream((fmt_gt % ex_path).str());

    // Setup subscriber
    BagSubscriber<sensor_msgs::Image> depth_img_sub, rgb_img_sub;
    BagSubscriber<sensor_msgs::NavSatFix> nav_sub;
    BagSubscriber<sensor_msgs::Imu> imu_sub;

    // Callback and sync
    // TimeSyncType sync(depth_img_sub, rgb_img_sub, nav_sub, imu_sub, 10);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(20), depth_img_sub, rgb_img_sub, nav_sub, imu_sub);
    sync.registerCallback(boost::bind(&WriteSyncData, _1, _2, _3, _4));

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.emplace_back(std::string("/camera/depth/image_raw"));
    topics.emplace_back(std::string("/camera/rgb/image_raw"));
    topics.emplace_back(std::string("/gnss_inertial/navsatfix"));
    topics.emplace_back(std::string("/gnss_inertial/imu"));

    rosbag::View bag_view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, bag_view){

        if(m.getTopic() == topics[0]){ // topic /camera/depth/image_raw
            sensor_msgs::Image::ConstPtr depth_ptr = m.instantiate<sensor_msgs::Image>();
            if(depth_ptr != NULL){
                depth_img_sub.CreateMsg(depth_ptr);
                // ROS_INFO("Depth msg created");
            }
        }
        if(m.getTopic() == topics[1]){ // topic /camera/rgb/image_raw
            sensor_msgs::Image::ConstPtr rgb_ptr = m.instantiate<sensor_msgs::Image>();
            if(rgb_ptr != NULL){
                rgb_img_sub.CreateMsg(rgb_ptr);
                // ROS_INFO("RGB msg created");
            }
        }
        if(m.getTopic() == topics[2]){ // topic /gnss_inertial/navsatfix
            sensor_msgs::NavSatFix::ConstPtr nav_ptr = m.instantiate<sensor_msgs::NavSatFix>();
            if(nav_ptr != NULL){
                nav_sub.CreateMsg(nav_ptr);
                // ROS_INFO("Nav msg created");
            }
        }
        if(m.getTopic() == topics[3]){ // topic /gnss_inertial/imu
            sensor_msgs::Imu::ConstPtr imu_ptr = m.instantiate<sensor_msgs::Imu>();
            if(imu_ptr != NULL){
                imu_sub.CreateMsg(imu_ptr);
                // ROS_INFO("IMU msg created");
            }
        }

        if(stop_flag){
            break;
        }
    }

    bag.close();

    return EXIT_SUCCESS;
}

void WriteSyncData(const sensor_msgs::Image::ConstPtr &depth_ptr,
                   const sensor_msgs::Image::ConstPtr &rgb_ptr,
                   const sensor_msgs::NavSatFix::ConstPtr &nav_ptr,
                   const sensor_msgs::Imu::ConstPtr &imu_ptr){

    static uint32_t frame_seq = 0;

    static boost::format fmt_depth("%s/depth_img/%06d.png");
    static boost::format fmt_rgb("%s/color_img/%06d.png");

    ROS_INFO("Wirting seq: %d", frame_seq);

    std::string depth_dst = (fmt_depth % ex_path % frame_seq).str();
    std::string rgb_dst = (fmt_rgb % ex_path % frame_seq).str();

    cv::imwrite(depth_dst, cv_bridge::toCvShare(depth_ptr)->image);
    cv::imwrite(rgb_dst, cv_bridge::toCvShare(rgb_ptr)->image);

    PoseData pose;
    PopulatePoseData(pose, frame_seq, nav_ptr, imu_ptr);
    CsvWritePose(gt_file, pose);
    
    frame_seq += 1;

    if(frame_seq > 500){
        stop_flag = true;
    }
}

void CsvWritePose(std::ofstream &ostream, const PoseData &data){
    static bool header_written = false;

    if(header_written == false){
        ostream << "seq, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w" << std::endl;
        header_written = true;
    }

    ostream << data.seq << "," << data.position_x << "," << data.position_y << "," << data.position_z 
                        << "," << data.quat_x << "," << data.quat_y << "," << data.quat_z << "," << data.quat_w << std::endl;
}

// Ref: https://github.com/ros-drivers/gps_umd/blob/master/gps_common/src/utm_odometry_node.cpp
void PopulatePoseData(PoseData &data, uint32_t seq, const sensor_msgs::NavSatFix::ConstPtr nav_ptr, const sensor_msgs::Imu::ConstPtr imu_ptr){
    double northing, easting;
    std::string zone;

    gps_common::LLtoUTM(nav_ptr->latitude, nav_ptr->longitude, northing, easting, zone);

    data.seq = seq;

    data.position_x = easting;
    data.position_y = northing;
    data.position_z = nav_ptr->altitude;

    data.quat_x = imu_ptr->orientation.x;
    data.quat_y = imu_ptr->orientation.y;
    data.quat_z = imu_ptr->orientation.z;
    data.quat_w = imu_ptr->orientation.w;
}