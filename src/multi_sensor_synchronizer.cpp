#include "multi_sensor_synchronizer.h"

MultiSensorSynchronizer::MultiSensorSynchronizer(const std::string& base_name) {
    start_time = std::chrono::steady_clock::now();
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();
    
    base_dir = base_name + "_" + timestamp;
    
    // Create directory structure
    std::filesystem::create_directories(base_dir);
    std::filesystem::create_directories(base_dir + "/realsense/color");
    std::filesystem::create_directories(base_dir + "/realsense/depth");
    std::filesystem::create_directories(base_dir + "/theta_cam0");
    std::filesystem::create_directories(base_dir + "/theta_cam1");
    std::filesystem::create_directories(base_dir + "/lidar_data");
    
    // Initialize CSV files
    sync_csv.open(base_dir + "/synchronized_data.csv");
    sync_csv << "sync_id,lidar_timestamp,realsense_timestamp,theta0_timestamp,theta1_timestamp,"
             << "lidar_realsense_diff_ms,lidar_theta0_diff_ms,lidar_theta1_diff_ms,"
             << "realsense_color,realsense_depth,theta0_image,theta1_image,num_points,has_imu\n";
    
    lidar_pointcloud_csv.open(base_dir + "/lidar_data/pointcloud_sync.csv");
    lidar_pointcloud_csv << "sync_id,timestamp,point_index,x,y,z,intensity,time,ring\n";
    
    lidar_imu_csv.open(base_dir + "/lidar_data/imu_sync.csv");
    lidar_imu_csv << "sync_id,timestamp,seq,stamp_sec,stamp_nsec,quat_x,quat_y,quat_z,quat_w,"
                  << "angular_vel_x,angular_vel_y,angular_vel_z,linear_acc_x,linear_acc_y,linear_acc_z\n";
    
    // Initialize UVC context
    uvc_error_t res = uvc_init(&uvc_ctx, NULL);
    if (res < 0) {
        std::cerr << "Failed to initialize UVC context" << std::endl;
        uvc_perror(res, "uvc_init");
        uvc_ctx = nullptr;
    }
    
    std::cout << "Recording to directory: " << base_dir << std::endl;
}

MultiSensorSynchronizer::~MultiSensorSynchronizer() {
    if (uvc_ctx) {
        uvc_exit(uvc_ctx);
    }
}

double MultiSensorSynchronizer::get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration<double>(duration).count();
}

void MultiSensorSynchronizer::cleanup_old_frames() {
    double current_time = get_current_time();
    double cutoff_time = current_time - buffer_duration;
    
    // Clean RealSense buffer
    {
        std::lock_guard<std::mutex> lock(realsense_mutex);
        realsense_buffer.erase(
            std::remove_if(realsense_buffer.begin(), realsense_buffer.end(),
                [cutoff_time](const TimestampedFrame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            realsense_buffer.end()
        );
    }
    
    // Clean THETA buffers
    for (int i = 0; i < active_theta_cameras; i++) {
        std::lock_guard<std::mutex> lock(theta_mutex[i]);
        theta_buffer[i].erase(
            std::remove_if(theta_buffer[i].begin(), theta_buffer[i].end(),
                [cutoff_time](const ThetaFrame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            theta_buffer[i].end()
        );
    }
    
    // Clean LiDAR buffer
    {
        std::lock_guard<std::mutex> lock(lidar_mutex);
        lidar_buffer.erase(
            std::remove_if(lidar_buffer.begin(), lidar_buffer.end(),
                [cutoff_time](const LidarFrame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            lidar_buffer.end()
        );
    }
}

int MultiSensorSynchronizer::find_theta_devices() {
    uvc_device_t **device_list;
    uvc_error_t res;
    int found = 0;
    
    res = uvc_get_device_list(uvc_ctx, &device_list);
    if (res != UVC_SUCCESS) {
        std::cerr << "Failed to get UVC device list" << std::endl;
        return 0;
    }
    
    std::cout << "Scanning for RICOH THETA X cameras (0x05ca:0x2717)..." << std::endl;
    
    for (int i = 0; device_list[i] != NULL && found < MAX_THETA_CAMERAS; i++) {
        uvc_device_descriptor_t *desc;
        if (uvc_get_device_descriptor(device_list[i], &desc) == UVC_SUCCESS) {
            if (desc->idVendor == 0x05ca && desc->idProduct == 0x2717) {
                uvc_ref_device(device_list[i]);
                if (theta_cameras[found].initialize(uvc_ctx, device_list[i], found)) {
                    found++;
                    std::cout << "Found THETA X camera " << found-1 << std::endl;
                } else {
                    uvc_unref_device(device_list[i]);
                }
            }
            uvc_free_device_descriptor(desc);
        }
    }
    
    uvc_free_device_list(device_list, 0);
    return found;
}

bool MultiSensorSynchronizer::initialize_theta_cameras() {
    if (!uvc_ctx) {
        std::cerr << "UVC context not initialized" << std::endl;
        return false;
    }
    
    active_theta_cameras = find_theta_devices();
    
    if (active_theta_cameras == 0) {
        std::cerr << "No RICOH THETA X cameras found" << std::endl;
        return false;
    }
    
    std::cout << "Found " << active_theta_cameras << " THETA X camera(s)" << std::endl;
    return true;
}

void MultiSensorSynchronizer::realsense_thread() {
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::context ctx;
        
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cout << "No RealSense devices found!" << std::endl;
            return;
        }
        
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        
        pipe.start(cfg);
        std::cout << "RealSense thread started" << std::endl;
        
        while (!shutdown) {
            try {
                rs2::frameset frames = pipe.wait_for_frames(100);
                rs2::depth_frame depth = frames.get_depth_frame();
                rs2::video_frame color = frames.get_color_frame();
                
                if (!depth || !color) continue;
                
                double capture_time = get_current_time();
                
                cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data());
                cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data());
                
                TimestampedFrame frame(capture_time, color_image, depth_image, "realsense");
                
                {
                    std::lock_guard<std::mutex> lock(realsense_mutex);
                    realsense_buffer.push_back(std::move(frame));
                }
                
            } catch (const rs2::error& e) {
                if (!shutdown) {
                    std::cout << "RealSense error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        pipe.stop();
    } catch (const std::exception& e) {
        std::cout << "RealSense thread failed: " << e.what() << std::endl;
    }
    std::cout << "RealSense thread stopped" << std::endl;
}

void MultiSensorSynchronizer::theta_collection_thread() {
    std::cout << "THETA collection thread started" << std::endl;
    
    // Start all THETA cameras
    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].start_streaming();
    }
    
    // Wait a moment for cameras to start
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    while (!shutdown) {
        try {
            // Collect latest frames from all THETA cameras
            for (int i = 0; i < active_theta_cameras; i++) {
                ThetaFrame frame = theta_cameras[i].get_latest_frame();
                if (frame.valid) {
                    // Create a new frame with current timestamp for better sync
                    ThetaFrame sync_frame = frame;
                    sync_frame.timestamp = get_current_time();
                    
                    {
                        std::lock_guard<std::mutex> lock(theta_mutex[i]);
                        theta_buffer[i].push_back(std::move(sync_frame));
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Higher frequency collection
            
        } catch (const std::exception& e) {
            if (!shutdown) {
                std::cout << "THETA collection error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    std::cout << "THETA collection thread stopped" << std::endl;
}

void MultiSensorSynchronizer::lidar_thread() {
    UnitreeLidarReader* lidar = nullptr;
    
    // Try different serial ports for LiDAR
    std::vector<std::string> lidar_ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
    std::string active_port;
    
    try {
        lidar = createUnitreeLidarReader();
        if (!lidar) {
            std::cout << "Failed to create LiDAR reader" << std::endl;
            return;
        }
        
        // Try each port until one works
        bool port_found = false;
        for (const auto& port : lidar_ports) {
            std::cout << "Trying LiDAR on port: " << port << std::endl;
            if (lidar->initializeSerial(port.c_str(), 4000000) == 0) {
                active_port = port;
                port_found = true;
                std::cout << "LiDAR connected on port: " << port << std::endl;
                break;
            } else {
                std::cout << "Failed to connect LiDAR on port: " << port << std::endl;
            }
        }
        
        if (!port_found) {
            std::cout << "Failed to initialize LiDAR on any port" << std::endl;
            return;
        }
        
        lidar->startLidarRotation();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        lidar->setLidarWorkMode(8);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "LiDAR thread started" << std::endl;
        
        LidarFrame current_frame;
        bool has_pending_cloud = false;
        int frame_id = 0;
        
        while (!shutdown) {
            try {
                int result = lidar->runParse();
                
                if (result == LIDAR_POINT_DATA_PACKET_TYPE) {
                    PointCloudUnitree cloud_data;
                    if (lidar->getPointCloud(cloud_data) && !cloud_data.points.empty()) {
                        if (has_pending_cloud) {
                            // Save the previous frame
                            {
                                std::lock_guard<std::mutex> lock(lidar_mutex);
                                lidar_buffer.push_back(std::move(current_frame));
                            }
                        }
                        
                        // Start new frame
                        current_frame = LidarFrame();
                        current_frame.timestamp = get_current_time();
                        current_frame.cloud_data = std::move(cloud_data);
                        current_frame.has_imu = false;
                        current_frame.frame_id = ++frame_id;
                        has_pending_cloud = true;
                    }
                }
                else if (result == LIDAR_IMU_DATA_PACKET_TYPE && has_pending_cloud) {
                    LidarImuData imu_data;
                    if (lidar->getImuData(imu_data)) {
                        current_frame.imu_data = imu_data;
                        current_frame.has_imu = true;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                
            } catch (const std::exception& e) {
                if (!shutdown) {
                    std::cout << "LiDAR error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
        
        // Save any pending frame
        if (has_pending_cloud) {
            std::lock_guard<std::mutex> lock(lidar_mutex);
            lidar_buffer.push_back(std::move(current_frame));
        }
        
    } catch (const std::exception& e) {
        std::cout << "LiDAR thread failed: " << e.what() << std::endl;
    }
    
    if (lidar) {
        try {
            delete lidar;
        } catch (...) {
            // Ignore cleanup errors
        }
    }
    std::cout << "LiDAR thread stopped" << std::endl;
}

// Main synchronization callback - LiDAR as anchor with rate limiting
bool MultiSensorSynchronizer::synchronization_callback() {
    if (shutdown) return false;
    
    cleanup_old_frames();
    
    // Get current data snapshots
    std::vector<LidarFrame> lidar_frames;
    std::vector<TimestampedFrame> realsense_frames;
    std::vector<ThetaFrame> theta0_frames, theta1_frames;
    
    {
        std::lock_guard<std::mutex> lock(lidar_mutex);
        lidar_frames = std::vector<LidarFrame>(lidar_buffer.begin(), lidar_buffer.end());
    }
    
    {
        std::lock_guard<std::mutex> lock(realsense_mutex);
        realsense_frames = std::vector<TimestampedFrame>(realsense_buffer.begin(), realsense_buffer.end());
    }
    
    if (active_theta_cameras > 0) {
        std::lock_guard<std::mutex> lock(theta_mutex[0]);
        theta0_frames = std::vector<ThetaFrame>(theta_buffer[0].begin(), theta_buffer[0].end());
    }
    
    if (active_theta_cameras > 1) {
        std::lock_guard<std::mutex> lock(theta_mutex[1]);
        theta1_frames = std::vector<ThetaFrame>(theta_buffer[1].begin(), theta_buffer[1].end());
    }
    
    // Find the MOST RECENT LiDAR frame as anchor
    if (lidar_frames.empty()) {
        return true; // No LiDAR data yet, continue
    }
    
    // Sort by timestamp and take the most recent
    auto latest_lidar = std::max_element(lidar_frames.begin(), lidar_frames.end(),
        [](const LidarFrame& a, const LidarFrame& b) {
            return a.timestamp < b.timestamp;
        });
    
    TimestampedFrame* best_realsense = nullptr;
    ThetaFrame* best_theta0 = nullptr;
    ThetaFrame* best_theta1 = nullptr;
    
    double rs_time_diff = std::numeric_limits<double>::max();
    double theta0_time_diff = std::numeric_limits<double>::max();
    double theta1_time_diff = std::numeric_limits<double>::max();
    
    // Find closest RealSense frame to the latest LiDAR frame
    for (auto& frame : realsense_frames) {
        double diff = std::abs(latest_lidar->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < rs_time_diff) {
            rs_time_diff = diff;
            best_realsense = &frame;
        }
    }
    
    // Find closest THETA 0 frame
    for (auto& frame : theta0_frames) {
        if (!frame.valid) continue; // Skip invalid frames
        double diff = std::abs(latest_lidar->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < theta0_time_diff) {
            theta0_time_diff = diff;
            best_theta0 = &frame;
        }
    }
    
    // Find closest THETA 1 frame
    for (auto& frame : theta1_frames) {
        if (!frame.valid) continue; // Skip invalid frames
        double diff = std::abs(latest_lidar->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < theta1_time_diff) {
            theta1_time_diff = diff;
            best_theta1 = &frame;
        }
    }
    
    // Save synchronized set if we have at least LiDAR + one other sensor
    if (best_realsense || best_theta0 || best_theta1) {
        save_synchronized_set(*latest_lidar, best_realsense, best_theta0, best_theta1,
                            rs_time_diff, theta0_time_diff, theta1_time_diff);
        
        // Remove ALL processed frames from buffers to prevent double-processing
        remove_processed_frames(*latest_lidar, best_realsense, best_theta0, best_theta1);
    }
    
    return true;
}

void MultiSensorSynchronizer::synchronization_loop() {
    std::cout << "Starting synchronization loop at " << sync_frequency.load() << " Hz..." << std::endl;
    
    auto last_sync_time = std::chrono::steady_clock::now();
    int actual_sync_count = 0;
    auto frequency_check_start = std::chrono::steady_clock::now();
    
    while (!shutdown) {
        auto callback_start = std::chrono::steady_clock::now();
        
        if (!synchronization_callback()) {
            break;
        }
        
        // Track actual synchronization frequency
        actual_sync_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_since_check = std::chrono::duration_cast<std::chrono::seconds>(now - frequency_check_start);
        
        // Report actual frequency every 10 seconds
        if (elapsed_since_check.count() >= 10) {
            double actual_freq = actual_sync_count / 10.0;
            std::cout << "Actual sync frequency: " << std::fixed << std::setprecision(2) 
                     << actual_freq << " Hz (target: " << sync_frequency.load() << " Hz)" << std::endl;
            actual_sync_count = 0;
            frequency_check_start = now;
        }
        
        // Calculate precise sleep time to maintain target frequency
        auto callback_end = std::chrono::steady_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
        
        // Target period in microseconds
        auto target_period_us = std::chrono::microseconds(1000000 / sync_frequency.load());
        
        if (callback_duration < target_period_us) {
            auto sleep_duration = target_period_us - callback_duration;
            std::unique_lock<std::mutex> lock(sync_mutex);
            shutdown_cv.wait_for(lock, sleep_duration, [this] { return shutdown.load(); });
        } else {
            // Callback took longer than target period
            if (sync_count % 20 == 0) {
                std::cout << "Warning: Sync callback taking " 
                         << callback_duration.count() / 1000.0 << " ms (target: " 
                         << target_period_us.count() / 1000.0 << " ms)" << std::endl;
            }
        }
    }
    
    std::cout << "Synchronization loop stopped" << std::endl;
}

void MultiSensorSynchronizer::remove_processed_frames(const LidarFrame& lidar_frame,
                           const TimestampedFrame* realsense_frame,
                           const ThetaFrame* theta0_frame,
                           const ThetaFrame* theta1_frame) {
    // Remove LiDAR frame and all older frames to prevent reprocessing
    {
        std::lock_guard<std::mutex> lock(lidar_mutex);
        lidar_buffer.erase(
            std::remove_if(lidar_buffer.begin(), lidar_buffer.end(),
                [&lidar_frame](const LidarFrame& frame) {
                    return frame.timestamp <= lidar_frame.timestamp;
                }),
            lidar_buffer.end()
        );
    }
    
    // Remove RealSense frame if used, and older frames
    if (realsense_frame) {
        std::lock_guard<std::mutex> lock(realsense_mutex);
        realsense_buffer.erase(
            std::remove_if(realsense_buffer.begin(), realsense_buffer.end(),
                [realsense_frame](const TimestampedFrame& frame) {
                    return frame.timestamp <= realsense_frame->timestamp;
                }),
            realsense_buffer.end()
        );
    }
    
    // Remove THETA frames if used, and older frames
    if (theta0_frame && active_theta_cameras > 0) {
        std::lock_guard<std::mutex> lock(theta_mutex[0]);
        theta_buffer[0].erase(
            std::remove_if(theta_buffer[0].begin(), theta_buffer[0].end(),
                [theta0_frame](const ThetaFrame& frame) {
                    return frame.timestamp <= theta0_frame->timestamp;
                }),
            theta_buffer[0].end()
        );
    }
    
    if (theta1_frame && active_theta_cameras > 1) {
        std::lock_guard<std::mutex> lock(theta_mutex[1]);
        theta_buffer[1].erase(
            std::remove_if(theta_buffer[1].begin(), theta_buffer[1].end(),
                [theta1_frame](const ThetaFrame& frame) {
                    return frame.timestamp <= theta1_frame->timestamp;
                }),
            theta_buffer[1].end()
        );
    }
}

void MultiSensorSynchronizer::save_synchronized_set(const LidarFrame& lidar_frame,
                         const TimestampedFrame* realsense_frame,
                         const ThetaFrame* theta0_frame,
                         const ThetaFrame* theta1_frame,
                         double rs_diff, double theta0_diff, double theta1_diff) {
    try {
        sync_count++;
        
        std::stringstream prefix_ss;
        prefix_ss << "sync_" << std::setfill('0') << std::setw(6) << sync_count;
        std::string prefix = prefix_ss.str();
        
        std::string rs_color_path = "", rs_depth_path = "";
        std::string theta0_path = "", theta1_path = "";
        
        // Save RealSense images if available
        if (realsense_frame) {
            rs_color_path = base_dir + "/realsense/color/" + prefix + "_color.png";
            rs_depth_path = base_dir + "/realsense/depth/" + prefix + "_depth.png";
            
            if (!realsense_frame->color_image.empty()) {
                cv::imwrite(rs_color_path, realsense_frame->color_image);
            }
            
            if (!realsense_frame->depth_image.empty()) {
                cv::Mat depth_8bit, depth_colormap;
                realsense_frame->depth_image.convertTo(depth_8bit, CV_8UC1, 0.03);
                cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                cv::imwrite(rs_depth_path, depth_colormap);
            }
        }
        
        // Save THETA images if available
        if (theta0_frame && theta0_frame->valid && !theta0_frame->image.empty()) {
            try {
                cv::Mat front_fisheye, back_fisheye;
                ThetaFisheyeConverter::convertThetaToFisheye(theta0_frame->image, 
                                                              front_fisheye, 
                                                              back_fisheye);
                
                std::string front_path = base_dir + "/theta_cam0/" + prefix + "_front_theta0.jpg";
                std::string back_path = base_dir + "/theta_cam0/" + prefix + "_back_theta0.jpg";
                
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(90);
                
                cv::imwrite(front_path, front_fisheye, compression_params);
                cv::imwrite(back_path, back_fisheye, compression_params);
                
                std::cout << "Saved THETA 0 frames: " 
                         << std::filesystem::path(front_path).filename().string() << ", "
                         << std::filesystem::path(back_path).filename().string() << std::endl;
                
                theta0_path = std::filesystem::path(front_path).filename().string() + " + " + 
                             std::filesystem::path(back_path).filename().string();
                
            } catch (const std::exception& e) {
                std::cerr << "Error converting THETA 0 image: " << e.what() << std::endl;
            }
        }
        
        if (theta1_frame && theta1_frame->valid && !theta1_frame->image.empty()) {
            try {
                cv::Mat front_fisheye, back_fisheye;
                ThetaFisheyeConverter::convertThetaToFisheye(theta1_frame->image, 
                                                              front_fisheye, 
                                                              back_fisheye);
                
                std::string front_path = base_dir + "/theta_cam1/" + prefix + "_front_theta1.jpg";
                std::string back_path = base_dir + "/theta_cam1/" + prefix + "_back_theta1.jpg";
                
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(90);
                
                cv::imwrite(front_path, front_fisheye, compression_params);
                cv::imwrite(back_path, back_fisheye, compression_params);
                
                std::cout << "Saved THETA 1 frames: " 
                         << std::filesystem::path(front_path).filename().string() << ", "
                         << std::filesystem::path(back_path).filename().string() << std::endl;
                
                theta1_path = std::filesystem::path(front_path).filename().string() + " + " + 
                             std::filesystem::path(back_path).filename().string();
                
            } catch (const std::exception& e) {
                std::cerr << "Error converting THETA 1 image: " << e.what() << std::endl;
            }
        }

        
        // Save LiDAR point cloud
        for (size_t i = 0; i < lidar_frame.cloud_data.points.size(); ++i) {
            const auto& point = lidar_frame.cloud_data.points[i];
            lidar_pointcloud_csv << sync_count << ","
                               << std::fixed << std::setprecision(6) << lidar_frame.timestamp << ","
                               << i << ","
                               << std::setprecision(4) << point.x << "," << point.y << "," << point.z << ","
                               << point.intensity << ","
                               << std::setprecision(6) << point.time << ","
                               << point.ring << "\n";
        }
        lidar_pointcloud_csv.flush();
        
        // Save IMU data if available
        if (lidar_frame.has_imu) {
            const auto& imu = lidar_frame.imu_data;
            lidar_imu_csv << sync_count << ","
                         << std::fixed << std::setprecision(6) << lidar_frame.timestamp << ","
                         << imu.info.seq << ","
                         << imu.info.stamp.sec << ","
                         << imu.info.stamp.nsec << ","
                         << std::setprecision(6)
                         << imu.quaternion[0] << "," << imu.quaternion[1] << ","
                         << imu.quaternion[2] << "," << imu.quaternion[3] << ","
                         << imu.angular_velocity[0] << "," << imu.angular_velocity[1] << ","
                         << imu.angular_velocity[2] << ","
                         << imu.linear_acceleration[0] << "," << imu.linear_acceleration[1] << ","
                         << imu.linear_acceleration[2] << "\n";
            lidar_imu_csv.flush();
        }
        
        // Write sync metadata
        sync_csv << sync_count << ","
                 << std::fixed << std::setprecision(6) << lidar_frame.timestamp << ","
                 << (realsense_frame ? std::to_string(realsense_frame->timestamp) : "") << ","
                 << (theta0_frame ? std::to_string(theta0_frame->timestamp) : "") << ","
                 << (theta1_frame ? std::to_string(theta1_frame->timestamp) : "") << ","
                 << std::setprecision(2) << (realsense_frame ? rs_diff * 1000 : -1) << ","
                 << (theta0_frame ? theta0_diff * 1000 : -1) << ","
                 << (theta1_frame ? theta1_diff * 1000 : -1) << ","
                 << std::filesystem::path(rs_color_path).filename().string() << ","
                 << std::filesystem::path(rs_depth_path).filename().string() << ","
                 << std::filesystem::path(theta0_path).filename().string() << ","
                 << std::filesystem::path(theta1_path).filename().string() << ","
                 << lidar_frame.cloud_data.points.size() << ","
                 << (lidar_frame.has_imu ? "true" : "false") << "\n";
        sync_csv.flush();
        
        if (sync_count % 5 == 0) {
            std::cout << "Synchronized " << sync_count << " sets at " << sync_frequency.load() << " Hz" << std::endl;
            
            // Print sensor status
            std::cout << "  LiDAR points: " << lidar_frame.cloud_data.points.size() << std::endl;
            if (realsense_frame) {
                std::cout << "  RealSense: OK" << std::endl;
            } else {
                std::cout << "  RealSense: No frame" << std::endl;
            }
            
            // Print THETA camera stats
            for (int i = 0; i < active_theta_cameras; i++) {
                std::cout << "  THETA " << i << ": " << theta_cameras[i].frames_received.load() 
                          << " frames, " << theta_cameras[i].samples_received.load() << " samples";
                if ((i == 0 && theta0_frame && theta0_frame->valid) || 
                    (i == 1 && theta1_frame && theta1_frame->valid)) {
                    std::cout << " [SYNCED]";
                } else {
                    std::cout << " [NO SYNC]";
                }
                std::cout << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error saving synchronized set: " << e.what() << std::endl;
    }
}

void MultiSensorSynchronizer::start_recording(int duration) {
    std::cout << "Starting multi-sensor recording for " << duration << " seconds..." << std::endl;
    std::cout << "Sync frequency: " << sync_frequency.load() << " Hz" << std::endl;
    std::cout << "Sync window: ±" << (sync_window * 1000) << " ms" << std::endl;
    
    // Initialize GStreamer
    gst_init(nullptr, nullptr);
    
    // Initialize THETA cameras
    if (!initialize_theta_cameras()) {
        std::cout << "Warning: No THETA cameras found, continuing with other sensors" << std::endl;
    }
    
    // Start all sensor threads
    realsense_thread_handle = std::thread(&MultiSensorSynchronizer::realsense_thread, this);
    lidar_thread_handle = std::thread(&MultiSensorSynchronizer::lidar_thread, this);
    
    // Start THETA collection thread if cameras are available
    std::thread theta_thread_handle;
    if (active_theta_cameras > 0) {
        theta_thread_handle = std::thread(&MultiSensorSynchronizer::theta_collection_thread, this);
    }
    
    // Start synchronization thread
    sync_thread = std::thread(&MultiSensorSynchronizer::synchronization_loop, this);
    
    // Wait for specified duration or until stopped
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        shutdown_cv.wait_for(lock, std::chrono::seconds(duration), [this] { return shutdown.load(); });
    }
    
    stop_recording();
    
    // Join THETA thread if it was started
    if (theta_thread_handle.joinable()) {
        theta_thread_handle.join();
    }
}

void MultiSensorSynchronizer::stop_recording() {
    std::cout << "Stopping recording..." << std::endl;
    shutdown = true;
    shutdown_cv.notify_all();
    
    // Cleanup THETA cameras
    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].cleanup();
    }
    
    // Join all threads
    if (sync_thread.joinable()) sync_thread.join();
    if (realsense_thread_handle.joinable()) realsense_thread_handle.join();
    if (lidar_thread_handle.joinable()) lidar_thread_handle.join();
    
    // Close files
    if (sync_csv.is_open()) sync_csv.close();
    if (lidar_pointcloud_csv.is_open()) lidar_pointcloud_csv.close();
    if (lidar_imu_csv.is_open()) lidar_imu_csv.close();
    
    std::cout << "Recording stopped. Data saved to: " << base_dir << std::endl;
    std::cout << "Total synchronized sets: " << sync_count << std::endl;
    std::cout << "Active THETA cameras: " << active_theta_cameras << std::endl;
    std::cout << "Output structure:" << std::endl;
    std::cout << "  - synchronized_data.csv (sync metadata)" << std::endl;
    std::cout << "  - realsense/ (color and depth images)" << std::endl;
    std::cout << "  - theta_cam0/ (THETA X camera 0 images)" << std::endl;
    std::cout << "  - theta_cam1/ (THETA X camera 1 images)" << std::endl;
    std::cout << "  - lidar_data/ (point clouds and IMU data)" << std::endl;
}

void MultiSensorSynchronizer::set_sync_frequency(int frequency) {
    if (frequency >= 1 && frequency <= 50) {
        sync_frequency = frequency;
        std::cout << "Sync frequency set to " << frequency << " Hz" << std::endl;
    }
}

void MultiSensorSynchronizer::set_sync_window(double window_ms) {
    sync_window = window_ms / 1000.0;
    std::cout << "Sync window set to ±" << window_ms << " ms" << std::endl;
}

int MultiSensorSynchronizer::get_sync_count() const {
    return sync_count;
}

int MultiSensorSynchronizer::get_active_theta_cameras() const {
    return active_theta_cameras;
}