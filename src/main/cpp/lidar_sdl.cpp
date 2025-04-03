#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "math.h"

#include <SDL3/SDL.h>

#ifndef FINDLINE_DEBUG
#define FINDLINE_DEBUG
#endif

#include "findline.h"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#define WIDTH 0.30

#define ARRAY_LEN(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

using namespace sl;

bool checkSLAMTECLIDARHealth(ILidarDriver *drv) {
	sl_result op_result;
	sl_lidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge
							   // whether the operation is succeed.
		printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, slamtec lidar internal error detected. "
							"Please reboot the device to retry.\n");
			// enable the following code if you want slamtec lidar to be reboot
			// by software drv->reset();
			return false;
		} else {
			return true;
		}

	} else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n",
				op_result);
		return false;
	}
}

bool cmpNode(const sl_lidar_response_measurement_node_hq_t &a,
			 const sl_lidar_response_measurement_node_hq_t &b) {
	return a.angle_z_q14 < b.angle_z_q14;
}

struct Camera {
	float scale;
	float zoom;
	int w, h;
	SDL_Renderer *rend;

	void set_color(Uint32 c) {
		SDL_SetRenderDrawColor(rend, (c >> 24) & 0xFF, (c >> 16) & 0xFF,
							   (c >> 8) & 0xFF, c & 0xFF);
	}

	float sx(float x) { return w / 2.0f - x * zoom; }

	float sy(float y) { return h / 2.0f - y * zoom; }

	void point(float x, float y, Uint32 color) {
		SDL_FRect rect = {
			.x = sx(y) - 5.0f,
			.y = sy(x) - 5.0f,
			.w = 10.0f,
			.h = 10.0f,
		};

		set_color(color);
		SDL_RenderFillRect(rend, &rect);
	}

	void line(float x0, float y0, float x1, float y1, Uint32 color) {
		set_color(color);
		SDL_RenderLine(rend, sx(y0), sy(x0), sx(y1), sy(x1));
	}
};

int main() {
	sl_u32 baudrate = 115200;
	IChannel *_channel;

#ifdef _WIN32
	const char *channel = "\\\\.\\com3";
#elif __APPLE__
	const char *channel = "/dev/tty.SLAB_USBtoUART";
#else
	const char *channel = "/dev/ttyUSB0";
#endif

	// create the driver instance
	ILidarDriver *drv = *createLidarDriver();

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	_channel = (*createSerialPortChannel(channel, baudrate));

	(drv)->connect(_channel);

	// Check lidar health.
	if (!checkSLAMTECLIDARHealth(drv)) {
		delete drv;
		return 1;
	}

	MotorCtrlSupport motorSupport;

	if (SL_IS_OK(drv->checkMotorCtrlSupport(motorSupport))) {
		printf("got support: %i\n", motorSupport);
	}

	LidarMotorInfo motor;

	if (SL_IS_OK(drv->getMotorInfo(motor))) {
		printf("motor: min=%i desired=%i max=%i ty=%i\n", motor.min_speed, motor.desired_speed, motor.max_speed, motor.motorCtrlSupport);
	}

	drv->setMotorSpeed(DEFAULT_MOTOR_SPEED);

	LidarScanMode selectedMode;
	std::vector<LidarScanMode> modes = {};

	printf("Scan modes:\n");

	if (SL_IS_OK(drv->getAllSupportedScanModes(modes))) {
		int best;
		float bestRate = INFINITY;

		for (const LidarScanMode& m : modes) {
			printf("%s: max dist=%fm us per sample=%f\n", m.scan_mode, m.max_distance, m.us_per_sample);

			if (m.us_per_sample < bestRate) {
				best = m.id;
				bestRate = m.us_per_sample;
			}
		}

		drv->startScanExpress(0, best, 0, &selectedMode);
	} else {
		drv->startScan(0, 1, 0, &selectedMode);
	}

	printf("scanning with mode %s: %fus per sample, %fm max distance\n", selectedMode.scan_mode, selectedMode.us_per_sample, selectedMode.max_distance);

	if (!SDL_Init(SDL_INIT_VIDEO)) {
		printf("Could not initialise SDL: %s", SDL_GetError());
		return 1;
	}

	SDL_Window *win =
		SDL_CreateWindow("Lidar demo", 800, 800,
						 SDL_WINDOW_HIGH_PIXEL_DENSITY | SDL_WINDOW_RESIZABLE);
	SDL_ShowWindow(win);

	SDL_Event ev;

	Camera cam = {.rend = SDL_CreateRenderer(win, NULL),
				  .scale = SDL_GetWindowDisplayScale(win),
				  .zoom = 1.0};

	SDL_SetRenderDrawBlendMode(cam.rend, SDL_BLENDMODE_BLEND);

	SDL_GetWindowSizeInPixels(win, &cam.w, &cam.h);

	sl_lidar_response_measurement_node_hq_t nodes[8192];
	DebugPoint pointBuf[8192];

	DebugBuffer dbg = {.points = pointBuf};

	Uint64 lastTick = 0;

	while (1) {
		// We should exit the timeout 20ms after the last tick.
		Uint64 deadline = lastTick + 20;
		Uint64 cur = SDL_GetTicks();

		// if (cur < deadline)
			SDL_WaitEventTimeout(NULL, 20);

		lastTick = cur;

		int redraw = 0;

		LineRes res;

		size_t count = ARRAY_LEN(nodes);

		switch (drv->grabScanDataHq(nodes, count, 0)) {
		case SL_RESULT_OK:
		case SL_RESULT_ALREADY_DONE: {
			float freq = 0.0;
			drv->getFrequency(selectedMode, nodes, count, freq);

			printf("frequency: %f\n", freq);

			for (size_t i = 0; i < count; i++)
				nodes[i].angle_z_q14 = (1 << 14) - nodes[i].angle_z_q14;

			drv->ascendScanData(nodes, count);

			if (find_line(WIDTH, nodes + count - 1, nodes, &res, &dbg) == -1) {
				printf("ERROR: line fitting failed\n");
				// return 0;
				break;
			};

			printf("cx=%f cy=%f rx=%f ry=%f\n", res.cx, res.cy, res.rx, res.ry);

			redraw = 1;
		}
		case SL_RESULT_OPERATION_TIMEOUT:
			break;
		default:
			printf("WARN: could not get scan data\n");
			break;
		};

		while (SDL_PollEvent(&ev)) {
			switch (ev.type) {
			case SDL_EVENT_MOUSE_WHEEL:
				cam.zoom *= exp(ev.wheel.y * 0.2);
				redraw = 1;
				break;

			case SDL_EVENT_WINDOW_RESIZED:
			case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
				SDL_GetWindowSizeInPixels(win, &cam.w, &cam.h);
				redraw = 1;
				break;

			case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
				cam.scale = SDL_GetWindowDisplayScale(win);
				redraw = 1;
				break;

			case SDL_EVENT_WINDOW_SHOWN:
			case SDL_EVENT_WINDOW_EXPOSED:
				redraw = 1;
				break;

			case SDL_EVENT_QUIT:
				goto end;
			default:
				break;
			}
		}

		if (!redraw)
			continue;

		SDL_SetRenderDrawColor(cam.rend, 0x18, 0x18, 0x18, 255);
		SDL_RenderClear(cam.rend);

		for (size_t i = 0; i < dbg.nPoints; i++) {
			DebugPoint p = dbg.points[i];
			cam.point(p.x, p.y, p.used ? 0x00FF00FF : 0xAAAAAAFF);
			cam.line(0, 0, p.x, p.y, 0xFF000080);
		}

		float y0 = -10.0;
		float y1 = 10.0;

		float x0 = dbg.m * y0 + dbg.b;
		float x1 = dbg.m * y1 + dbg.b;

		cam.line(x0, y0, x1, y1, 0x0000FFFF);

		cam.line(0, 0, res.cx, res.cy, 0xFFFF00FF);
		cam.line(0, 0, res.rx * 5.0, res.ry * 5.0, 0xFFFF00FF);

		cam.line(0, 0, 1.0, 0.0, 0xFFFFFFFF);

		SDL_RenderPresent(cam.rend);
	}

end:
	SDL_DestroyRenderer(cam.rend);
	SDL_DestroyWindow(win);
	SDL_Quit();

	drv->stop();
	SDL_Delay(200);
	drv->setMotorSpeed(0);

	delete drv;

	return 0;
}
