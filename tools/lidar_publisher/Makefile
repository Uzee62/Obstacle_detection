# tools/lidar_publisher/Makefile
#
# Build the Slamtec-SDK-backed scan publisher. The Python driver
# (maritime_perception.sensors.lidar.driver) spawns this binary as a
# subprocess and consumes scans from its stdout.
#
# Requires the Slamtec RPLidar SDK to be cloned AND built at the path
# given by SDK_DIR. Default is ~/rplidar_sdk; override on the make
# command line if your tree lives elsewhere:
#
#     make SDK_DIR=/opt/rplidar_sdk

SDK_DIR ?= $(HOME)/rplidar_sdk
SDK_LIB  = $(SDK_DIR)/output/Linux/Release/libsl_lidar_sdk.a
SDK_INC  = -I$(SDK_DIR)/sdk/include -I$(SDK_DIR)/sdk/src

CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++14 -Wall -Wno-unused-function
LDFLAGS  ?= -lpthread

BIN = lidar_publisher

all: $(BIN)

$(BIN): main.cpp $(SDK_LIB)
	$(CXX) $(CXXFLAGS) $(SDK_INC) -o $@ main.cpp $(SDK_LIB) $(LDFLAGS)

$(SDK_LIB):
	@echo "ERROR: Slamtec SDK static library not found at:" >&2
	@echo "  $(SDK_LIB)" >&2
	@echo "" >&2
	@echo "Build the SDK first:" >&2
	@echo "  git clone https://github.com/Slamtec/rplidar_sdk.git $(SDK_DIR)" >&2
	@echo "  (cd $(SDK_DIR) && make)" >&2
	@echo "" >&2
	@echo "Or override SDK_DIR if it lives elsewhere:" >&2
	@echo "  make SDK_DIR=/path/to/rplidar_sdk" >&2
	@exit 1

clean:
	rm -f $(BIN)

.PHONY: all clean
