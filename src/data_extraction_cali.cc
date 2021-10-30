#include <iostream>

#include <boost/format.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <gps_common/conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <livox_ros_driver/CustomMsg.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

using TimeSyncType = message_filters::TimeSynchronizer<livox_ros_driver::CustomMsg, sensor_msgs::Image, sensor_msgs::NavSatFix, sensor_msgs::Imu>;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, sensor_msgs::Image, sensor_msgs::NavSatFix, sensor_msgs::Imu>;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>{
public:
    void CreateMsg(const boost::shared_ptr<M const> &msg){this->signalMessage(msg);}
};

void SaveSyncData(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr,
                  const sensor_msgs::Image::ConstPtr &rgb_kptr,
                  const sensor_msgs::NavSatFix::ConstPtr &nav_kptr,
                  const sensor_msgs::Imu::ConstPtr &imu_kptr);

Eigen::Isometry3d Gnss2Isometry(const sensor_msgs::NavSatFix::ConstPtr &nav_kptr,
                                const sensor_msgs::Imu::ConstPtr &imu_kptr);
livox_ros_driver::CustomMsg::Ptr PtcloudTransform(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr, const Eigen::Isometry3d &Transform);
livox_ros_driver::CustomPoint PtTransforms(const livox_ros_driver::CustomPoint &pt, const Eigen::Isometry3d &Transform);

namespace{
    double record_duration_s = 10.0;

    Eigen::Isometry3d Tbl;
}

int main(int argc, char const *argv[]){
    if(argc != 3){
        ROS_ERROR("Usage: rosrun camvox_data_extraction <path-to-rosbag> <path-to-extraction-dir>('/'not included)");
        return EXIT_FAILURE;
    }

    Tbl.matrix() << 0.0610474,  -0.997654,  -0.0309794, 0.503,
                    0.998116,   0.0608269,  0.00801179, 1.166,
                    -0.0061086, -0.0314102, 0.999488,   1.388,
                    0.0       , 0.0       , 0.0     ,   1.0;

    ros::Time::init();

    std::string bag_path(argv[1]);
    std::string ex_path(argv[2]);

    // Setup output file stream
    boost::format fmt_outbag("%s/cali.bag");

    // Setup subscriber
    BagSubscriber<livox_ros_driver::CustomMsg> lidar_sub;
    BagSubscriber<sensor_msgs::Image> rgb_img_sub;
    BagSubscriber<sensor_msgs::NavSatFix> nav_sub;
    BagSubscriber<sensor_msgs::Imu> imu_sub;

    // Callback and sync
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), lidar_sub, rgb_img_sub, nav_sub, imu_sub);
    sync.registerCallback(boost::bind(&SaveSyncData, _1, _2, _3, _4));

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.emplace_back(std::string("/livox/lidar"));
    topics.emplace_back(std::string("/camera/rgb/image_raw"));
    topics.emplace_back(std::string("/gnss_inertial/navsatfix"));
    topics.emplace_back(std::string("/gnss_inertial/imu"));

    rosbag::View bag_view(bag, rosbag::TopicQuery(topics));

    bool first_msg = true;
    double start_time = 0.0;

    BOOST_FOREACH(rosbag::MessageInstance const m, bag_view){
        if(first_msg){
            start_time = m.getTime().toSec();
            first_msg = false;
        }else if(m.getTime().toSec() - start_time > record_duration_s){
            break;
        }

        if(m.getTopic() == topics[0]){ // topic /camera/depth/image_raw
            livox_ros_driver::CustomMsg::ConstPtr ptcloud_kptr = m.instantiate<livox_ros_driver::CustomMsg>();
            if(ptcloud_kptr != NULL){
                lidar_sub.CreateMsg(ptcloud_kptr);
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
    }

    return EXIT_SUCCESS;
}

void SaveSyncData(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr,
                  const sensor_msgs::Image::ConstPtr &rgb_kptr,
                  const sensor_msgs::NavSatFix::ConstPtr &nav_kptr,
                  const sensor_msgs::Imu::ConstPtr &imu_kptr){
    using Eigen::Isometry3d;

    static bool is_first = true;
    static Isometry3d first_pose;

    if(is_first){
        first_pose = Gnss2Isometry(nav_kptr, imu_kptr);
        is_first = false;
    }

    // Transform from lidar to world
    Isometry3d Twl = first_pose.inverse() * Gnss2Isometry(nav_kptr, imu_kptr) * Tbl;
}

livox_ros_driver::CustomMsg::Ptr PtcloudTransform(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr, const Eigen::Isometry3d &Transform){
    livox_ros_driver::CustomMsg res_cloud = *ptcloud_kptr;
    for(size_t i = 0; i < res_cloud.points.size(); ++i){
        res_cloud.points[i] = PtTransforms(res_cloud.points[i], Transform);
    }
    
    return boost::make_shared<livox_ros_driver::CustomMsg>(res_cloud);
}

livox_ros_driver::CustomPoint PtTransforms(const livox_ros_driver::CustomPoint &pt, const Eigen::Isometry3d &Transform){
    using Eigen::Vector3d;

    Vector3d pt3d(pt.x, pt.y, pt.z);
    Vector3d transed_pt3d = Transform * pt3d;

    livox_ros_driver::CustomPoint res_pt = pt;
    res_pt.x = transed_pt3d.x();
    res_pt.y = transed_pt3d.y();
    res_pt.z = transed_pt3d.z();

    return res_pt;
}

Eigen::Isometry3d Gnss2Isometry(const sensor_msgs::NavSatFix::ConstPtr &nav_kptr,
                                const sensor_msgs::Imu::ConstPtr &imu_kptr){
    using Eigen::Isometry3d;
    using Eigen::Vector3d;
    using Eigen::Quaterniond;

    double northing, easting;
    std::string zone;
    gps_common::LLtoUTM(nav_kptr->latitude, nav_kptr->longitude, northing, easting, zone);

    Vector3d trans(easting, northing, nav_kptr->altitude);
    Quaterniond orient(imu_kptr->orientation.w, imu_kptr->orientation.x, imu_kptr->orientation.y, imu_kptr->orientation.z);

    Isometry3d iso;
    iso.pretranslate(trans).prerotate(orient);

    return iso;
}