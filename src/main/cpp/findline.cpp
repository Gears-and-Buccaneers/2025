#include "findline.h"
#include <math.h>

// Sums points `a` and `b` and writes the result to `p`.
void point_add(Point* a, Point* b, Point* p) {
	p->x   = a->x   + b->x;
	p->y   = a->y   + b->y;
	p->x_x = a->x_x + b->x_x;
	p->y_y = a->y_y + b->y_y;
	p->x_y = a->x_y + b->x_y;
}

// Finds the square distance between points.
double point_dist_sq(Point* a, Point* b) {
	double xd = a->x - b->x;
	double yd = a->y - b->y;

	return xd * xd + yd * yd;
}

// Converts a lidar measurement into a point. Returns 1 if this point should be skipped.
int measure_to_point(const sl_lidar_response_measurement_node_hq_t* m, Point* p) {
	if (m->quality == 0) return 1;

	double a = (m->angle_z_q14 / 16384.0) * M_PI_2;
	double d = (m->dist_mm_q2 / 4.0) / 1000.0;

	p->x = sin(a) * d;
	p->y = cos(a) * d;

	p->x_x = p->x * p->x;
	p->y_y = p->y * p->y;
	p->x_y = p->x * p->y;

	return 0;
}

// Stores the slope and y-intercept of a line.
typedef struct {
	double m, b;
} LsrBuf;

// Perform least-squares regression on a parameter sum, and find the R^2 value.
void lsr(Point* s, int n, LsrBuf* b) {
	b->m = (n * s->x_y - s->x * s->y) / (n * s->x_x - s->x * s->x);
	b->b = (s->y - b->m * s->x) / n;
}

double eval(LsrBuf* line, double x) {
	return line->m * x + line->b;
}

// Find the error between a point an a regression line.
double err(Point* p, LsrBuf* line) {
	return fabs(p->y - eval(line, p->x));
}

// Derives parameters for the best-fitting line directly in front of the sensor.
// Values of `left` are read descending until the `lend` parameter, and values in `right` are
// read in ascending order up to the `rend` parameter. Returns -1 on error, and 0 on success.
int find_line(
	double width,
	sl_lidar_response_measurement_node_hq_t* left,
	sl_lidar_response_measurement_node_hq_t* right,
	LineRes* res
	#ifdef FINDLINE_DEBUG
	, DebugBuffer* dbg
	#endif
) {
	// The parameter sum in the window.
	Point sum;
	LsrBuf line;
	// The current points on the left and right side.
	Point cl, cr;
	// The next points on the left and right side.
	Point nl, nr;
	// Buffers for the parameter sums for the left and right side.
	Point sl, sr;

	// Pop the first points from the left and right sides.
	do if (left == right) return -1; while (measure_to_point( left--, &cl));
	do if (left == right) return -1; while (measure_to_point(right++, &cr));

	#ifdef FINDLINE_DEBUG
	dbg->points[0] = { .x = cl.x, .y = cl.y, .used = 1 };
	dbg->points[1] = { .x = cr.x, .y = cr.y, .used = 1 };
	#endif

	// Initialise the sum buffer with these two points.
	point_add(&cl, &cr, &sum);
	int n = 2;

	// Initialise the left and right starting points.
	do if (left == right) return -1; while (measure_to_point( left--, &nl));
	do if (left == right) return -1; while (measure_to_point(right++, &nr));

	do {
		// Add the left and right points to the current sum.
		point_add(&sum, &nl, &sl);
		point_add(&sum, &nr, &sr);
		// Increment the set size.
		n += 1;

		// Accept the point with the lower error to the regression line.
		if (err(&nl, &line) < err(&nr, &line)) {
			sum = sl;
			cl = nl;

			do if (left == right) return -1; while (measure_to_point( left--, &nl));

			#ifdef FINDLINE_DEBUG
			dbg->points[n - 1] = { .x = cl.x, .y = cl.y, .used = 1 };
			#endif
		} else {
			sum = sr;
			cr = nr;

			do if (left == right) return -1; while (measure_to_point(right++, &nr));

			#ifdef FINDLINE_DEBUG
			dbg->points[n - 1] = { .x = cr.x, .y = cr.y, .used = 1 };
			#endif
		}

		lsr(&sum, n, &line);
	} while (left != right && (point_dist_sq(&nl, &cr) < width * width || point_dist_sq(&cl, &nr) < width * width));

	// Find the center x and y coordinates.
	res->cx = (cl.x + cr.x) / 2;
	res->cy = eval(&line, res->cx);

	// Find the vector of the line.
	res->rx = 1.0 / hypot(line.m, 1.0);
	res->ry = line.m * res->rx;

	#ifdef FINDLINE_DEBUG
	while (left != right) {
		if (!measure_to_point(right++, &cr)) {
			dbg->points[n++] = { .x = cr.x, .y = cr.y, .used = 0 };
		}
	}

	dbg->m = line.m;
	dbg->b = line.b;

	dbg->nPoints = n;
	#endif

	return 0;
}
