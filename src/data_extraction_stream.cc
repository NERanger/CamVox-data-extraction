#include <string>
#include <vector>
#include <iomanip>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <gps_common/conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "livox_ros_driver/CustomMsg.h"

#include "point_type/LivoxPoint.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

using SyncPolicyLidar = message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, sensor_msgs::NavSatFix, sensor_msgs::Imu>;
using SyncPolicyImg = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::NavSatFix, sensor_msgs::Imu>;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>{
public:
    void CreateMsg(const boost::shared_ptr<M const> &msg){this->signalMessage(msg);}
};

struct PoseData{
    uint64_t timestamp;  // NanoSec
    double position_x;
    double position_y;
    double position_z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};

namespace{
    const std::string kImgTopicName("/camera/rgb/image_raw");
    const std::string kLidarPtTopicName("/livox/lidar");
    const std::string kLidarImuTopicName("/livox/imu");
    const std::string kGtImuTopicName("/gnss_inertial/imu");
    const std::string kGtNavsatTopicName("/gnss_inertial/navsatfix");

    bool seq_first_write = true;

    std::ofstream gt_file;
}

void WriteSeqInfo(std::ofstream &ofstream, uint64_t timestamp, const std::string &data_type, const std::string &location);
void WriteSeqInfo(std::ofstream &ofstream, sensor_msgs::Imu::ConstPtr imu_kptr);
void LidarGtSyncCallback(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr,
                         const sensor_msgs::NavSatFix::ConstPtr &gtnav_kptr,
                         const sensor_msgs::Imu::ConstPtr &gtimu_kptr);
void ImgGtSyncCallback(const sensor_msgs::Image::ConstPtr &img_kptr,
                         const sensor_msgs::NavSatFix::ConstPtr &gtnav_kptr,
                         const sensor_msgs::Imu::ConstPtr &gtimu_kptr);
                         
void PopulatePoseData(PoseData &data, uint64_t timestamp, const sensor_msgs::NavSatFix::ConstPtr nav_ptr, const sensor_msgs::Imu::ConstPtr imu_ptr);
void WriteGtPose(std::ofstream &ofstream, const PoseData &data);
// void WriteImuInfo(std::ofstream &ofstream, sensor_msgs::Imu::ConstPtr imu_kptr);

int main(int argc, char const *argv[]){
    
    // Desired structure of extraction directory
    // -- extraction (extration directory)
    // ----| lidar/
    // ----| img/
    // ----| sequence.csv
    // ----| groundtruth.csv

    if(argc != 3){
        ROS_ERROR("Usage: rosrun camvox_data_extraction data_extraction_stream <path-to-rosbag> <path-to-extraction-dir>");
        return EXIT_FAILURE;
    }

    ros::Time::init();

    std::string bag_path(argv[1]);
    std::string ex_path(argv[2]);

    // Remove '/'
    if(ex_path.back() == '/'){
        ex_path.pop_back();
    }

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // Extract data (no groundtruth) in the first run
    std::vector<std::string> data_topics;
    data_topics.emplace_back(kImgTopicName);
    data_topics.emplace_back(kLidarPtTopicName);
    data_topics.emplace_back(kLidarImuTopicName);
    rosbag::View data_view(bag, rosbag::TopicQuery(data_topics));

    // Path in format
    boost::format fmt_rgb("%s/img/%d.png");
    boost::format fmt_ptcloud("%s/lidar/%d.pcd");
    boost::format fmt_seq("%s/sequence.csv");

    std::ofstream seq_file((fmt_seq % ex_path).str());

    unsigned int img_idx = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, data_view){
        if(m.getTopic() == kImgTopicName){
            sensor_msgs::Image::ConstPtr img_kptr = m.instantiate<sensor_msgs::Image>();
            if(img_kptr != NULL){
                boost::format relpath("img/%d.png");

                uint64_t timestamp = img_kptr->header.stamp.toNSec();

                std::string dst = (fmt_rgb % ex_path % timestamp).str();
                ROS_INFO_STREAM("Extracting rgb   msg, timestamp: " << timestamp);
                ROS_INFO_STREAM("Extracing destination: " << dst);
                ROS_INFO_STREAM("Current img seq: " << img_idx++);

                cv_bridge::CvImageConstPtr cvimg_kptr = cv_bridge::toCvShare(img_kptr);
                cv::cvtColor(cvimg_kptr->image, cvimg_kptr->image, cv::COLOR_RGB2BGR);
                cv::imwrite(dst, cvimg_kptr->image);
                WriteSeqInfo(seq_file, timestamp, "img", (relpath % timestamp).str());
            }
        }

        if(m.getTopic() == kLidarPtTopicName){
            livox_ros_driver::CustomMsg::ConstPtr ptcloud_kptr = m.instantiate<livox_ros_driver::CustomMsg>();
            if(ptcloud_kptr != NULL){
                boost::format relpath("lidar/%d.pcd");

                uint64_t timestamp = ptcloud_kptr->header.stamp.toNSec();

                std::string dst = (fmt_ptcloud % ex_path % timestamp).str();
                ROS_INFO_STREAM("Extracting lidar msg, timestamp: " << timestamp);
                ROS_INFO_STREAM("Extracing destination: " << dst);

                // pcl::PointCloud<pcl::PointXYZ> cloud;
                // for(uint32_t i = 0; i < ptcloud_kptr->point_num; ++i){
                //     cloud.push_back(pcl::PointXYZ(ptcloud_kptr->points[i].x, 
                //                                   ptcloud_kptr->points[i].y, 
                //                                   ptcloud_kptr->points[i].z));
                // }
                // cloud.height = 1;
                // cloud.width = cloud.size();
                // pcl::io::savePCDFileBinary(dst, cloud);
                pcl::PointCloud<LivoxPoint> cloud;
                for(uint32_t i = 0; i < ptcloud_kptr->point_num; ++i){
                    uint64_t pt_timestamp = (uint64_t)ptcloud_kptr->points[i].offset_time + ptcloud_kptr->timebase;
                    cloud.push_back(LivoxPoint{ptcloud_kptr->points[i].x,
                                               ptcloud_kptr->points[i].y,
                                               ptcloud_kptr->points[i].z,
                                               ptcloud_kptr->points[i].reflectivity,
                                               (uint32_t)(pt_timestamp >> 32),
                                               (uint32_t)pt_timestamp});
                }
                cloud.height = 1;
                cloud.width = cloud.size();
                pcl::io::savePCDFileBinary(dst, cloud);

                WriteSeqInfo(seq_file, timestamp, "lidar", (relpath % timestamp).str());
            }
        }

        if(m.getTopic() == kLidarImuTopicName){
            sensor_msgs::Imu::ConstPtr imu_kptr = m.instantiate<sensor_msgs::Imu>();
            if(imu_kptr != NULL){
                // WriteImuInfo(imu_file, imu_kptr);
                WriteSeqInfo(seq_file, imu_kptr);
            }
        }
    }

    img_idx = 0;

    // Extract groundtruth in the second run
    std::vector<std::string> gt_topics;
    gt_topics.emplace_back(kImgTopicName);
    gt_topics.emplace_back(kLidarPtTopicName);
    gt_topics.emplace_back(kGtNavsatTopicName);
    gt_topics.emplace_back(kGtImuTopicName);
    rosbag::View gt_view(bag, rosbag::TopicQuery(gt_topics));

    // Setup output file stream
    boost::format fmt_gt("%s/groundtruth.csv");
    gt_file = std::ofstream((fmt_gt % ex_path).str());

    // Setup subscriber
    BagSubscriber<sensor_msgs::Image> img_sub;
    BagSubscriber<livox_ros_driver::CustomMsg> lidar_sub;
    BagSubscriber<sensor_msgs::NavSatFix> gtnav_sub;
    BagSubscriber<sensor_msgs::Imu> gtimu_sub;

    // Callback and sync
    message_filters::Synchronizer<SyncPolicyLidar> lidar_gt_sync(SyncPolicyLidar(100), lidar_sub, gtnav_sub, gtimu_sub);
    message_filters::Synchronizer<SyncPolicyImg> img_gt_sync(SyncPolicyImg(100), img_sub, gtnav_sub, gtimu_sub);
    lidar_gt_sync.registerCallback(boost::bind(&LidarGtSyncCallback, _1, _2, _3));
    img_gt_sync.registerCallback(boost::bind(&ImgGtSyncCallback, _1, _2, _3));

    ROS_INFO_STREAM("--- Start groundtruth extraction ---");

    BOOST_FOREACH(rosbag::MessageInstance const m, gt_view){
        if(m.getTopic() == kImgTopicName){
            sensor_msgs::Image::ConstPtr img_kptr = m.instantiate<sensor_msgs::Image>();
            if(img_kptr != NULL){
                img_sub.CreateMsg(img_kptr);
                ROS_INFO_STREAM("Current img seq: " << img_idx++);
            }
        }

        if(m.getTopic() == kLidarPtTopicName){
            livox_ros_driver::CustomMsg::ConstPtr ptcloud_kptr = m.instantiate<livox_ros_driver::CustomMsg>();
            if(ptcloud_kptr != NULL){
                lidar_sub.CreateMsg(ptcloud_kptr);
            }
        }

        if(m.getTopic() == kGtNavsatTopicName){
            sensor_msgs::NavSatFix::ConstPtr gtnav_kptr = m.instantiate<sensor_msgs::NavSatFix>();
            if(gtnav_kptr != NULL){
                gtnav_sub.CreateMsg(gtnav_kptr);
            }
        }

        if(m.getTopic() == kGtImuTopicName){
            sensor_msgs::Imu::ConstPtr gtimu_kptr = m.instantiate<sensor_msgs::Imu>();
            if(gtimu_kptr != NULL){
                gtimu_sub.CreateMsg(gtimu_kptr);
            }
        }
    }

    seq_file.close();
    gt_file.close();
    bag.close();
    
    return EXIT_SUCCESS;
}

void WriteSeqInfo(std::ofstream &ofstream, uint64_t timestamp, const std::string &data_type, const std::string &location){
    if(seq_first_write){
        ofstream << "timestamp [ns],data_type,location" 
                 << ",w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1],"
                 << "a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]" << std::endl;
        seq_first_write = false;
    }

    ofstream << timestamp << "," << data_type << "," << location << "," 
             << "-,-,-,-,-,-" << std::endl;
}

void WriteSeqInfo(std::ofstream &ofstream, sensor_msgs::Imu::ConstPtr imu_kptr){
    if(seq_first_write){
        ofstream << "timestamp [ns],data_type,location" 
                 << ",w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1],"
                 << "a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]" << std::endl;
        seq_first_write = false;
    }

    ofstream << imu_kptr->header.stamp.toNSec() << ",imu,-," << std::setprecision(15)
             << imu_kptr->angular_velocity.x << "," << imu_kptr->angular_velocity.y << "," << imu_kptr->angular_velocity.z << ","
             << imu_kptr->linear_acceleration.x << "," << imu_kptr->linear_acceleration.y << "," << imu_kptr->linear_acceleration.z
             << std::endl;
}

void LidarGtSyncCallback(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr,
                         const sensor_msgs::NavSatFix::ConstPtr &gtnav_kptr,
                         const sensor_msgs::Imu::ConstPtr &gtimu_kptr){
    PoseData pose;
    PopulatePoseData(pose, ptcloud_kptr->header.stamp.toNSec(), gtnav_kptr, gtimu_kptr);
    WriteGtPose(gt_file, pose);

}

void ImgGtSyncCallback(const sensor_msgs::Image::ConstPtr &img_kptr,
                       const sensor_msgs::NavSatFix::ConstPtr &gtnav_kptr,
                       const sensor_msgs::Imu::ConstPtr &gtimu_kptr){
    PoseData pose;
    PopulatePoseData(pose, img_kptr->header.stamp.toNSec(), gtnav_kptr, gtimu_kptr);
    WriteGtPose(gt_file, pose);
}

// Ref: https://github.com/ros-drivers/gps_umd/blob/master/gps_common/src/utm_odometry_node.cpp
void PopulatePoseData(PoseData &data, uint64_t timestamp, const sensor_msgs::NavSatFix::ConstPtr nav_ptr, const sensor_msgs::Imu::ConstPtr imu_ptr){
    double northing, easting;
    std::string zone;

    gps_common::LLtoUTM(nav_ptr->latitude, nav_ptr->longitude, northing, easting, zone);

    data.timestamp = timestamp;

    data.position_x = easting;
    data.position_y = northing;
    data.position_z = nav_ptr->altitude;

    data.quat_x = imu_ptr->orientation.x;
    data.quat_y = imu_ptr->orientation.y;
    data.quat_z = imu_ptr->orientation.z;
    data.quat_w = imu_ptr->orientation.w;
}

void WriteGtPose(std::ofstream &ofstream, const PoseData &data){
    static bool header_written = false;

    if(header_written == false){
        ofstream << "timestamp, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w" << std::endl;
        header_written = true;
    }

    // Be sure to set the output precision
    ofstream << std::setprecision(15) << data.timestamp << "," << data.position_x << "," << data.position_y << "," << data.position_z 
                        << "," << data.quat_x << "," << data.quat_y << "," << data.quat_z << "," << data.quat_w << std::endl;
}

void WriteImuInfo(std::ofstream &ofstream, sensor_msgs::Imu::ConstPtr imu_kptr){
    static bool first_write = true;

    if(first_write){
        ofstream << "timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1]," <<
                    "a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]" << std::endl;
        first_write = false;
    }

    ofstream << imu_kptr->header.stamp.toNSec() << ","
             << imu_kptr->angular_velocity.x << "," << imu_kptr->angular_velocity.y << "," << imu_kptr->angular_velocity.z << ","
             << imu_kptr->linear_acceleration.x << "," << imu_kptr->linear_acceleration.y << "," << imu_kptr->linear_acceleration.z
             << std::endl;
}