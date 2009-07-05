#ifndef ELLIPSE_H
#define ELLIPSE_H

#include "common.h"
#include <vector>
#include <complex>

#define ST_UNDEFINED 	0
#define ST_DEFINED		1

class satellipse {
  public:
	vector<complex<double> > _trace;
	satellipse(){};
	
	void add_position(complex<double> new_position, int time_step);
	unsigned int known_time_steps();
	complex<double> get_pos_at(int time_step);
};

#endif