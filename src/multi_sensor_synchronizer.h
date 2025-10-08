#ifndef MULTI_SENSOR_SYNCHRONIZER_H
#define MULTI_SENSOR_SYNCHRONIZER_H

#include "common.h"
#include "sensor_data.h"
#include "theta_camera.h"
#include "equirect_to_fisheye.h"

class MultiSensorSynchronizer {
private:
    std::string base_dir;
    
    // Sensor data buffers
    std::deque<TimestampedFrame> realsense_buffer;
    std::deque<ThetaFrame> theta_buffer[MAX_THETA_CAMERAS];
    std::deque<LidarFrame> lidar_buffer;
    
    // Thread synchronization
    std::mutex realsense_mutex;
    std::mutex theta_mutex[MAX_THETA_CAMERAS];
    std::mutex lidar_mutex;
    std::mutex sync_mutex;
    std::condition_variable shutdown_cv;
    std::atomic<bool> shutdown{false};
    
    // THETA cameras
    uvc_context_t *uvc_ctx;
    ThetaCamera theta_cameras[MAX_THETA_CAMERAS];
    int active_theta_cameras = 0;
    
    // Output files
    std::ofstream sync_csv;
    std::ofstream lidar_pointcloud_csv;
    std::ofstream lidar_imu_csv;
    
    // Configuration
    double sync_window = 0.05; // 50ms
    double buffer_duration = 0.5;
    std::atomic<int> sync_frequency{5}; // Hz
    int sync_count = 0;
    
    // Timing
    std::chrono::steady_clock::time_point start_time;
    std::thread sync_thread;
    std::thread realsense_thread_handle;
    std::thread lidar_thread_handle;

    // Private methods
    double get_current_time();
    void cleanup_old_frames();
    int find_theta_devices();
    bool synchronization_callback();
    void synchronization_loop();
    void remove_processed_frames(const LidarFrame& lidar_frame,
                               const TimestampedFrame* realsense_frame,
                               const ThetaFrame* theta0_frame,
                               const ThetaFrame* theta1_frame);
    void save_synchronized_set(const LidarFrame& lidar_frame,
                             const TimestampedFrame* realsense_frame,
                             const ThetaFrame* theta0_frame,
                             const ThetaFrame* theta1_frame,
                             double rs_diff, double theta0_diff, double theta1_diff);
    
    // Thread functions
    void realsense_thread();
    void theta_collection_thread();
    void lidar_thread();

public:
    MultiSensorSynchronizer(const std::string& base_name = "multi_sensor_data");
    ~MultiSensorSynchronizer();
    
    bool initialize_theta_cameras();
    void start_recording(int duration = 30);
    void stop_recording();
    void set_sync_frequency(int frequency);
    void set_sync_window(double window_ms);
    int get_sync_count() const;
    int get_active_theta_cameras() const;
};

#endif // MULTI_SENSOR_SYNCHRONIZER_H