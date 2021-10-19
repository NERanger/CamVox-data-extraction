#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct LivoxPoint{
    float x;
    float y;
    float z;
    uint8_t reflectivity; // 0~255
    uint32_t timestamp_h;  // In nano second
    uint32_t timestamp_l;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, reflectivity, reflectivity)
                                  (uint32_t, timestamp_h, timestamp_h)
                                  (uint32_t, timestamp_l, timestamp_l)
)