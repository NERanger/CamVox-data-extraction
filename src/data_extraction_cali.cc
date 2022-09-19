#include <iostream>
#include <iomanip>

#include <boost/format.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <gps_common/conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

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
pcl::PointCloud<pcl::PointXYZI>::Ptr ToPclCloud(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr);
void SaveGtPoses();

namespace{
    double record_duration_s = 5.0;

    Eigen::Isometry3d Tbl;

    std::string bag_path;
    std::string output_path;

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> gt_poses;
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

    bag_path = std::string(argv[1]);
    output_path = std::string(argv[2]);

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

    SaveGtPoses();

    return EXIT_SUCCESS;
}

void SaveSyncData(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr,
                  const sensor_msgs::Image::ConstPtr &rgb_kptr,
                  const sensor_msgs::NavSatFix::ConstPtr &nav_kptr,
                  const sensor_msgs::Imu::ConstPtr &imu_kptr){
    using Eigen::Isometry3d;

    static bool is_first = true;
    static Isometry3d first_pose;

    static boost::format fmt_lidar("%s/livox/%06d.pcd");
    static boost::format fmt_img("%s/image/%06d.png");

    static unsigned int data_idx = 0;

    if(is_first){
        first_pose = Gnss2Isometry(nav_kptr, imu_kptr);
        is_first = false;
    }

    // Transform from lidar to world
    Isometry3d Twl = first_pose.inverse() * Gnss2Isometry(nav_kptr, imu_kptr) * Tbl;
    gt_poses.push_back(Twl);

    // Save ptcloud
    std::string lidar_file = (fmt_lidar % output_path % data_idx).str();
    ROS_INFO_STREAM(lidar_file);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = ToPclCloud(ptcloud_kptr);
    pcl::io::savePCDFile(lidar_file, *cloud);

    // Save image
    std::string img_file = (fmt_img % output_path % data_idx).str();
    ROS_INFO_STREAM(img_file);
    cv::Mat rgb_img = cv_bridge::toCvShare(rgb_kptr)->image;
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
    cv::imwrite(img_file, gray_img);

    data_idx += 1;

    ROS_INFO_STREAM(Twl.matrix());

}

void SaveGtPoses(){
    ROS_ASSERT(!gt_poses.empty());
    
    static boost::format fmt_gt("%s/groundtruth.txt");

    std::ofstream gt_file((fmt_gt % output_path).str());
    for(const Eigen::Isometry3d &pose : gt_poses){
        const Eigen::Matrix4d mat = pose.matrix();

        gt_file << std::setprecision(5);

        gt_file        << mat(0, 0) << " " << mat(0, 1) << " " << mat(0, 2) << " " << mat(0, 3)
                << " " << mat(1, 0) << " " << mat(1, 1) << " " << mat(1, 2) << " " << mat(1, 3)
                << " " << mat(2, 0) << " " << mat(2, 1) << " " << mat(2, 2) << " " << mat(2, 3);
        
        gt_file << std::endl;
    }
    
    gt_file.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ToPclCloud(const livox_ros_driver::CustomMsg::ConstPtr &ptcloud_kptr){
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i = 0; i < ptcloud_kptr->points.size(); ++i){
        const livox_ros_driver::CustomPoint &pt = ptcloud_kptr->points.at(i);
        pcl::PointXYZI pt_xyzi;
        pt_xyzi.x = pt.x;
        pt_xyzi.y = pt.y;
        pt_xyzi.z = pt.z;
        pt_xyzi.intensity = pt.reflectivity;

        out->push_back(pt_xyzi);
    }

    return out;
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

    Isometry3d iso(orient);
    iso.pretranslate(trans);

    return iso;
}