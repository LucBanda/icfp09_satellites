#ifndef ELLIPSE_H
#define ELLIPSE_H
#include "bin.h"

class ellipse {
	public:
    Complex semi_major_axis;
	double period;
	Complex apexf;
	Complex apexs;
    double excentricity;
    Complex center;
    Complex radii;
    double rotate;

	ellipse(int instance, int target);
	double distance(Complex pos);
	//Complex get_speed(Complex pos);
};

#endif