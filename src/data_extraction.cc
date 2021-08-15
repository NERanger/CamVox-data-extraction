#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

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

    boost::format fmt_rgb("%s/color_img/%06d.png");
    boost::format fmt_depth("%s/depth_img/%06d.png");

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
    
    return EXIT_SUCCESS;
}
