#include "multi_sensor_synchronizer.h"

// Global synchronizer for signal handler
MultiSensorSynchronizer* g_synchronizer = nullptr;

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping recording..." << std::endl;
    if (g_synchronizer) {
        g_synchronizer->stop_recording();
    }
    exit(0);
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -d, --duration SECONDS    Recording duration (default: 30)" << std::endl;
    std::cout << "  -f, --frequency HZ        Sync frequency (default: 5)" << std::endl;
    std::cout << "  -w, --window MS           Sync window in milliseconds (default: 50)" << std::endl;
    std::cout << "  -o, --output NAME         Output directory prefix (default: multi_sensor_data)" << std::endl;
    std::cout << "  -h, --help                Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << program_name << " -d 60 -f 10 -w 30 -o experiment_001" << std::endl;
}

int main(int argc, char* argv[]) {
    int duration = 30;
    int frequency = 5;
    double sync_window_ms = 50.0;
    std::string output = "multi_sensor_data";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if ((arg == "-d" || arg == "--duration") && i + 1 < argc) {
            duration = std::stoi(argv[++i]);
        } else if ((arg == "-f" || arg == "--frequency") && i + 1 < argc) {
            frequency = std::stoi(argv[++i]);
        } else if ((arg == "-w" || arg == "--window") && i + 1 < argc) {
            sync_window_ms = std::stod(argv[++i]);
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            output = argv[++i];
        } else {
            std::cout << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Validate parameters
    if (duration <= 0 || frequency <= 0 || frequency > 50 || sync_window_ms <= 0) {
        std::cout << "Invalid parameters!" << std::endl;
        print_usage(argv[0]);
        return 1;
    }
    
    std::cout << "=== Multi-Sensor Synchronization with LiDAR Anchor ===" << std::endl;
    std::cout << "Sensors:" << std::endl;
    std::cout << "  - Unitree 4D LiDAR L2 (anchor)" << std::endl;
    std::cout << "  - Intel RealSense Depth Camera" << std::endl;
    std::cout << "  - RICOH THETA X 360° Cameras (auto-detected)" << std::endl;
    std::cout << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Duration: " << duration << " seconds" << std::endl;
    std::cout << "  Sync frequency: " << frequency << " Hz" << std::endl;
    std::cout << "  Sync window: ±" << sync_window_ms << " ms" << std::endl;
    std::cout << "  Output: " << output << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop early" << std::endl;
    std::cout << std::endl;
    
    MultiSensorSynchronizer synchronizer(output);
    synchronizer.set_sync_frequency(frequency);
    synchronizer.set_sync_window(sync_window_ms);
    g_synchronizer = &synchronizer;
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        synchronizer.start_recording(duration);
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        synchronizer.stop_recording();
        return 1;
    }
    
    return 0;
}