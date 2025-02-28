#ifndef FINDLINE_H
#define FINDLINE_H

#include "sl_lidar_cmd.h"

typedef struct {
	// The rectangular coordinates of the point.
	double x, y;
	// The squares of the coordinates.
	double x_x, y_y;
	// The product of the coordinates.
	double x_y;
} Point;

typedef struct {
	// The coordinates of the centerpoint of the line.
	double cx, cy;
	// The normalised vector of the relative line direction.
	double rx, ry;
} LineRes;

#ifdef FINDLINE_DEBUG
typedef struct {
	double x;
	double y;
	int used;
} DebugPoint;

typedef struct {
	DebugPoint* points;
	size_t nPoints;
	double m;
	double b;
} DebugBuffer;
#endif

int find_line(
	double width,
	sl_lidar_response_measurement_node_hq_t* left,
	sl_lidar_response_measurement_node_hq_t* right,
	LineRes* res
	#ifdef FINDLINE_DEBUG
	, DebugBuffer* dbg
	#endif
);

#endif
