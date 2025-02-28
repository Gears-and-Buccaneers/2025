all: build/render ../deploy/liblidar.so

exec: build/render
	build/render

build/render: render.cpp findline.h findline.cpp
	g++ -std=c++23 -Wall -Wextra -o build/render -DFINDLINE_DEBUG render.cpp findline.cpp -I/opt/homebrew/include -I./rplidar_sdk/sdk/include -L./rplidar_sdk/output/Darwin/Release -lsl_lidar_sdk -lSDL3
