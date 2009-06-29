#ifndef MEET_AND_GREED_H
#define MEET_AND_GREED_H

#include "common.h"
#include "controller.h"
#include <complex>



class meetandgreed : public Icontroller {
  private:
	address _vy_addr;
	address _vx_addr;
	address _sat_x_addr;
	address _sat_y_addr;
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	int ignition_time;
	int time_to_stop;
	double speed_back_x;
	double speed_back_y;

	complex<double> old_sat_abs_pos;
    complex<double> old_my_abs_pos;
    
	satellite *me;
	satellite *target;
	
	double max_angular_speed;
	void calculate_action(double *dvx, double *dvy, uint32_t time_step);
    bool rectified;
	int delay;
  public:
	meetandgreed(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};


#endif
