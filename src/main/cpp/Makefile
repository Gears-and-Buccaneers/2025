all: build/lidar_sdl ../deploy/liblidar.so

rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a:
	cd rplidar_sdk && PATH=$(PATH):/Users/devenspeyrer/Downloads/roborio-academic/bin BUILD_TARGET_PLATFORM=Linux CROSS_COMPILE_PREFIX=arm-frc2025-linux-gnueabi ./cross_compile.sh

rplidar_sdk/output/Darwin/Release/libsl_lidar_sdk.a:
	cd rplidar_sdk && make

# ../deploy/liblidar.so: lidar_jni.cpp findline.cpp lidar_jni.h rplidar_sdk/output/Darwin/Release/libsl_lidar_sdk.a
# 	g++ -shared -fpic -std=c++23 -Wall -Wextra -o ../deploy/liblidar.so lidar_jni.cpp findline.cpp -I/opt/homebrew/opt/openjdk/include -I./rplidar_sdk/sdk/include -L./rplidar_sdk/output/Darwin/Release -lsl_lidar_sdk

../deploy/liblidar.so: lidar_jni.cpp findline.cpp lidar_jni.h rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
	./roborio-academic/bin/arm-frc2025-linux-gnueabi-g++ -shared -fpic -std=c++23 -Wall -Wextra -o ../deploy/liblidar.so lidar_jni.cpp findline.cpp -I./gradle-jni/src/main/resources/arm-linux-jni/linux -I./gradle-jni/src/main/resources/arm-linux-jni -I./rplidar_sdk/sdk/include -L./rplidar_sdk/output/Linux/Release -lsl_lidar_sdk

build/lidar_sdl: lidar_sdl.cpp findline.h findline.cpp
	g++ -std=c++23 -Wall -Wextra -o build/lidar_sdl -DFINDLINE_DEBUG lidar_sdl.cpp findline.cpp -I/opt/homebrew/include -I./rplidar_sdk/sdk/include -L./rplidar_sdk/output/Darwin/Release -lsl_lidar_sdk -lSDL3
