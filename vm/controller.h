#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "vm_state.h"
#include "trace_generator.h"


class Icontroller {
  protected:
	trace_generator * _trace;
	double _instance;
	address _instance_addr;
	address _score_addr;
	address _fuel_addr;
	
  public :
	Icontroller(trace_generator* trace, double instance) : _trace(trace), _instance(instance){};
	virtual bool step (uint32_t time_step) = 0;
	virtual void monitor() = 0;
};

class hohmann : public Icontroller {
  private:
	address _vy_addr;
	address _vx_addr;
	address _target_orbit_addr;
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	uint32_t ignition_time;
	uint32_t time_to_stop;
	double speed_back_x;
	double speed_back_y;
	
	void calculate_action(double *dvx, double *dvy, uint32_t time_step);
	
  public:
	hohmann(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};
#endif