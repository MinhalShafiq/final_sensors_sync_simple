#include <iostream>
#include "logitech_camera.h"

LogitechCamera::LogitechCamera() : initialized(false), width(3840), height(2160) {
}

LogitechCamera::~LogitechCamera() {
    cleanup();
}

bool LogitechCamera::initialize(const std::string& device, int w, int h) {
    if (initialized) {
        return true;
    }

    device_path = device;
    width = w;
    height = h;

    // Open the camera device
    cap.open(device_path);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open Logitech camera at " << device_path << std::endl;
        return false;
    }

    // Set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_AUTOFOCUS, 0);  // Disable autofocus
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    // Verify settings
    double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    if (actual_width != width || actual_height != height) {
        std::cout << "Warning: Camera resolution mismatch. Requested: " << width << "x" << height 
                  << " Got: " << actual_width << "x" << actual_height << std::endl;
    }

    // Read a test frame to verify camera is working
    cv::Mat test_frame;
    if (!cap.read(test_frame)) {
        std::cerr << "Failed to capture test frame from camera" << std::endl;
        cleanup();
        return false;
    }

    initialized = true;
    std::cout << "Logitech camera initialized successfully at " << width << "x" << height << std::endl;
    return true;
}

bool LogitechCamera::capture_frame(cv::Mat& frame) {
    if (!initialized || !cap.isOpened()) {
        return false;
    }

    if (!cap.read(frame)) {
        std::cerr << "Failed to capture frame from Logitech camera" << std::endl;
        return false;
    }

    return !frame.empty();
}

void LogitechCamera::cleanup() {
    if (cap.isOpened()) {
        cap.release();
    }
    initialized = false;
}