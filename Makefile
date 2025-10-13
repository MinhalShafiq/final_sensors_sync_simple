CXX = g++
CXXFLAGS = -std=c++17 -pthread

# GStreamer / LibUVC flags
GST_CFLAGS  = $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 libuvc)
GST_LIBS    = $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 libuvc)

# OpenCV (try opencv4 first, fallback to opencv)
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv)
OPENCV_LIBS   = $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv)

# RealSense (optional)
REALSENSE_CFLAGS = $(shell pkg-config --cflags realsense2 2>/dev/null || echo "")
REALSENSE_LIBS   = $(shell pkg-config --libs realsense2 2>/dev/null || echo "-lrealsense2")

# Unitree LiDAR SDK
UNITREE_SDK_PATH = unilidar_sdk
UNITREE_INCLUDE  = -I$(UNITREE_SDK_PATH)/include
UNITREE_LIB      = $(UNITREE_SDK_PATH)/lib/x86_64/libunitree_lidar_sdk.a

# Combined flags
ALL_CFLAGS = $(CXXFLAGS) $(GST_CFLAGS) $(OPENCV_CFLAGS) $(REALSENSE_CFLAGS) $(UNITREE_INCLUDE)
ALL_LIBS   = $(GST_LIBS) $(OPENCV_LIBS) $(REALSENSE_LIBS) $(UNITREE_LIB) -lpthread

# Project setup
TARGET = all_sensors_sync
SRC_DIR = src
SOURCES = $(SRC_DIR)/main.cpp \
          $(SRC_DIR)/multi_sensor_synchronizer.cpp \
          $(SRC_DIR)/theta_camera.cpp \
		  $(SRC_DIR)/logitech_camera.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: $(TARGET)

# Link final executable
$(TARGET): $(OBJECTS)
	@echo "Linking $(TARGET)..."
	$(CXX) $(OBJECTS) -o $@ $(ALL_LIBS)

# Compile sources to objects
%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) $(ALL_CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)

# Run executable
run: $(TARGET)
	./$(TARGET)

# Debug: print pkg-config flags
test-flags:
	@echo "GStreamer:    $(GST_CFLAGS)"
	@echo "GStreamerApp: $(shell pkg-config --cflags gstreamer-app-1.0)"
	@echo "LibUVC:       $(shell pkg-config --cflags libuvc)"
	@echo "OpenCV:       $(OPENCV_CFLAGS)"
	@echo "RealSense:    $(REALSENSE_CFLAGS)"
	@echo "Unitree:      $(UNITREE_INCLUDE)"

.PHONY: all clean run test-flags
