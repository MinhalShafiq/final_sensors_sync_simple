#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "common.h"

// Sensor data structures
struct TimestampedFrame {
    double timestamp;
    cv::Mat color_image;
    cv::Mat depth_image;
    std::string source_id;
    
    TimestampedFrame() = default;
    TimestampedFrame(double ts, const cv::Mat& color, const cv::Mat& depth, const std::string& id) 
        : timestamp(ts), color_image(color.clone()), depth_image(depth.clone()), source_id(id) {}
};

struct ThetaFrame {
    double timestamp;
    cv::Mat image;
    std::string camera_id;
    bool valid;
    
    ThetaFrame() : valid(false) {}
    ThetaFrame(double ts, const cv::Mat& img, const std::string& id)
        : timestamp(ts), image(img.clone()), camera_id(id), valid(true) {}
};

struct LidarFrame {
    double timestamp;
    PointCloudUnitree cloud_data;
    LidarImuData imu_data;
    bool has_imu;
    int frame_id;
    
    LidarFrame() : has_imu(false), frame_id(0) {}
};

#endif // SENSOR_DATA_H