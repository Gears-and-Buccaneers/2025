all: build/lidar_sdl ../deploy/liblidar.so

build/lidar_sdl: lidar_sdl.cpp findline.h findline.cpp
	g++ -std=c++23 -Wall -Wextra -o build/lidar_sdl -DFINDLINE_DEBUG lidar_sdl.cpp findline.cpp -I/opt/homebrew/include -I./rplidar_sdk/sdk/include -L./rplidar_sdk/output/Darwin/Release -lsl_lidar_sdk -lSDL3
