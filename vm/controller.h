#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "vm_state.h"
#include "trace_generator.h"
#include "satellite.h"

class Icontroller {
  protected:
	trace_generator * _trace;
	double _instance;
	address _instance_addr;
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	address _score_addr;
	address _fuel_addr;
	
  public :
	Icontroller(trace_generator* trace) : _trace(trace){};
	virtual bool step (uint32_t time_step) = 0;
	virtual void monitor() = 0;
};

class hohmann : public Icontroller {
  private:
	satellite *me;
	complex<double> calculate_action(uint32_t time_step);
	
  public:
	hohmann(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};
#endif