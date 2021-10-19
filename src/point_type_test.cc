#include <iostream>

#include "point_type/LivoxPoint.hpp"

int main(int argc, char const *argv[]){
    uint64_t dummy_timestamp = 1234567898765;

    pcl::PointCloud<LivoxPoint> cloud;
    cloud.resize (2);
    cloud.width = 2;
    cloud.height = 1;

    cloud[0].x = 1;
    cloud[0].timestamp_h = static_cast<uint32_t>(dummy_timestamp >> 32);
    cloud[0].timestamp_l = static_cast<uint32_t>(dummy_timestamp);

    uint64_t restored_timestamp = ((uint64_t)cloud[0].timestamp_h << 32) | cloud[0].timestamp_l;

    std::cout << "restored timestamp: " << restored_timestamp << std::endl;

    return 0;
}
