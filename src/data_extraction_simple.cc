#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "livox_ros_driver/CustomMsg.h"

int main(int argc, char const *argv[]){
    
    // Desired structure of extraction directory
    // -- camvox_dataset (extration directory)
    // ----| color_img
    // ----| depth_img

    if(argc != 3){
        ROS_ERROR("Usage: rosrun camvox_data_extraction <path-to-rosbag> <path-to-extraction-dir>('/'not included)");
        return EXIT_FAILURE;
    }

    std::string bag_path(argv[1]);
    std::string ex_path(argv[2]);

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // std::vector<std::string> topics;
    // topics.emplace_back("/isee_depth");
    // topics.emplace_back("/isee_rgb");

    rosbag::View rgb_view(bag, rosbag::TopicQuery("/isee_rgb"));
    rosbag::View depth_view(bag, rosbag::TopicQuery("/isee_depth"));
    rosbag::View ptcloud_view(bag, rosbag::TopicQuery("/livox/lidar"));

    boost::format fmt_rgb("%s/color_img/%06d.png");
    boost::format fmt_depth("%s/depth_img/%06d.png");
    boost::format fmt_ptcloud("%s/ptcloud/%06d.pcd");

    BOOST_FOREACH(const rosbag::MessageInstance m, rgb_view){
        sensor_msgs::Image::ConstPtr ptr = m.instantiate<sensor_msgs::Image>();
        if(ptr != NULL){
            std::string dst = (fmt_rgb % ex_path % ptr->header.seq).str();
            ROS_INFO("Extracting rgb msg, seq number: %d", ptr->header.seq);
            ROS_INFO("Extracing destination: %s", dst.c_str());
            cv_bridge::CvImageConstPtr cvimg_ptr = cv_bridge::toCvShare(ptr);
            cv::imwrite(dst, cvimg_ptr->image);
        }
    }

    // Depth image should be type CV_16U
    // Depth factor: 504
    BOOST_FOREACH(const rosbag::MessageInstance m, depth_view){
        sensor_msgs::Image::ConstPtr ptr = m.instantiate<sensor_msgs::Image>();
        if(ptr != NULL){
            std::string dst = (fmt_depth % ex_path % ptr->header.seq).str();
            ROS_INFO("Extracting depth msg, seq number: %d", ptr->header.seq);
            ROS_INFO("Extracing destination: %s", dst.c_str());
            cv_bridge::CvImageConstPtr cvimg_ptr = cv_bridge::toCvShare(ptr);
            cv::imwrite(dst, cvimg_ptr->image);
        }
    }

    BOOST_FOREACH(const rosbag::MessageInstance m, ptcloud_view){
        livox_ros_driver::CustomMsg::ConstPtr ptr = m.instantiate<livox_ros_driver::CustomMsg>();
        if(ptr != NULL){
            std::string dst = (fmt_ptcloud % ex_path % ptr->header.seq).str();
            ROS_INFO("Extracting ptcloud msg, seq number: %d, point num: %d", ptr->header.seq, ptr->point_num);
            ROS_INFO("Extracing destination: %s", dst.c_str());

            pcl::PointCloud<pcl::PointXYZ> cloud;
            for(uint32_t i = 0; i < ptr->point_num; ++i){
                cloud.push_back(pcl::PointXYZ(ptr->points[i].x, ptr->points[i].y, ptr->points[i].z));
            }
            cloud.height = 1;
            cloud.width = cloud.size();

            pcl::io::savePCDFileBinary(dst, cloud);
        }
    }
    
    return EXIT_SUCCESS;
}
