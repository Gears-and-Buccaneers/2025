#include "findline.h"
#include <math.h>

typedef struct {
	// The rectangular coordinates of the point.
	double x, y;
	// The squares of the coordinates.
	double y_y;
	// The product of the coordinates.
	double x_y;
	// The sine and cosine of the angle to the point.
	double sin, cos;
	// The distance to the point.
	double dist;
} Point;

// Adds point `b` to point `a`.
void point_add(Point* a, Point* b) {
	a->x   += b->x;
	a->y   += b->y;
	a->x_y += b->x_y;
	a->y_y += b->y_y;
}

// Converts a lidar measurement into a point. Returns 1 if this point should be skipped.
int measure_to_point(const sl_lidar_response_measurement_node_hq_t* m, Point* p) {
	if (m->quality == 0) return 1;

	double a = (m->angle_z_q14 / 16384.0) * M_PI_2;
	p->dist = (m->dist_mm_q2 / 4.0) / 1000.0;

	p->cos = cos(a);
	p->sin = sin(a);

	p->x = p->cos * p->dist;
	p->y = p->sin * p->dist;

	p->x_y = p->x * p->y;
	p->y_y = p->y * p->y;

	return 0;
}

// Stores the slope and y-intercept of a line.
typedef struct {
	double m, fact, b;
} LsrBuf;

// Perform least-squares regression on a parameter sum.
void lsr(Point* s, int n, LsrBuf* b) {
	b->m = (n * s->x_y - s->x * s->y) / (n * s->y_y - s->y * s->y);
	b->b = (s->x - b->m * s->y) / n;

	b->fact = b->b * sqrt(b->m * b->m + 1);
}

double dist(LsrBuf* line, Point* p) {
	return line->b / (line->m * p->sin + p->cos);
}

void eval(LsrBuf* line, Point* p, double* x, double* y) {
	double d = dist(line, p);

	*x = -d * p->sin;
	*y =  d * p->cos;
}

// Find the error between a point an a regression line.
double err(LsrBuf* line, Point* p) {
	double d = line->b / (line->m * p->sin + p->cos);
	return fabs(p->dist - dist(line, p));
}

double projected_dist(Point* a, Point* b, LsrBuf* line) {
	double af = a->sin / (line->m * a->sin + a->cos);
	double bf = b->sin / (line->m * b->sin + b->cos);

	return abs(line->fact * (af - bf));
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

	// Pop the first points from the left and right sides.
	do if (left == right) return -1; while (measure_to_point( left--, &cl));
	do if (left == right) return -1; while (measure_to_point(right++, &cr));

	#ifdef FINDLINE_DEBUG
	dbg->points[0] = { .x = cl.x, .y = cl.y, .used = 1 };
	dbg->points[1] = { .x = cr.x, .y = cr.y, .used = 1 };
	#endif

	// Initialise the sum buffer with these two points.
	sum = cl;
	point_add(&sum, &cr);
	int n = 2;

	lsr(&sum, n, &line);

	// Initialise the left and right starting points.
	do if (left == right) return -1; while (measure_to_point( left--, &nl));
	do if (left == right) return -1; while (measure_to_point(right++, &nr));

	do {
		// Increment the set size.
		n += 1;
		// Accept the point with the lower error to the regression line.
		if (err(&line, &nl) < err(&line, &nr)) {
			point_add(&sum, &nl);
			cl = nl;

			do if (left == right) return -1; while (measure_to_point( left--, &nl));

			#ifdef FINDLINE_DEBUG
			dbg->points[n - 1] = { .x = cl.x, .y = cl.y, .used = 1 };
			#endif
		} else {
			point_add(&sum, &nr);
			cr = nr;

			do if (left == right) return -1; while (measure_to_point(right++, &nr));

			#ifdef FINDLINE_DEBUG
			dbg->points[n - 1] = { .x = cr.x, .y = cr.y, .used = 1 };
			#endif
		}

		lsr(&sum, n, &line);
	} while (projected_dist(&nl, &cr, &line) < width || projected_dist(&cl, &nr, &line) < width);

	// Find the center x and y coordinates.
	double ax, ay, bx, by;

	eval(&line, &cl, &ax, &ay);
	eval(&line, &cr, &bx, &by);

	res->cx = (ax + bx) / 2;
	res->cy = (ay + by) / 2;

	// Find the vector of the line.
	res->rx = 1.0 / sqrt(line.m * line.m + 1);
	res->ry = -line.m * res->rx;

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
