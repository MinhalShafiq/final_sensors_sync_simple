#ifndef LOGITECH_CAMERA_H
#define LOGITECH_CAMERA_H

#include "sensor_data.h"

struct LogitechFrame {
    double timestamp;
    cv::Mat image;
    bool valid;
    
    LogitechFrame() : valid(false) {}
    LogitechFrame(double ts, const cv::Mat& img)
        : timestamp(ts), image(img.clone()), valid(true) {}
};

class LogitechCamera {
private:
    cv::VideoCapture cap;
    std::string device_path;
    int width;
    int height;
    bool initialized;
    
public:
    LogitechCamera();
    ~LogitechCamera();
    
    bool initialize(const std::string& device = "/dev/video8", 
                   int w = 3840, int h = 2160);
    bool capture_frame(cv::Mat& frame);
    void cleanup();
    bool is_initialized() const { return initialized; }
};

#endif // LOGITECH_CAMERA_H