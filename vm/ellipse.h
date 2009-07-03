#ifndef ELLIPSE_H
#define ELLIPSE_H

#include "common.h"
#include <vector>
#include <complex>

#define ST_UNDEFINED 	0
#define ST_DEFINED		1

class satellipse {
  public:
	
	complex<double> _apogee;
	complex<double> _perige;
	complex<double> _center;
	double 			_period;
	vector<complex<double> > _trace;
	int last_time_orig;
	complex<double> _great_axe;
	complex<double> _little_axe;
	int _state;
  
	satellipse() : _apogee(0,0), _perige(1E10,1E10), _center(0,0){};
	
	void add_position(complex<double> new_position, int time_step);
	bool is_defined();
	complex<double> position_in(complex<double> current_position, int current_time_step, int step_forward);
	
};

#endif