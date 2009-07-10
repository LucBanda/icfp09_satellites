#ifndef EXCENTRIC_H
#define EXCENTRIC_H

#include "common.h"
#include "controller.h"

class excentric : public Icontroller {
  private:
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	satellite *me;
	satellite *target;
	
	complex<double> calculate_action(uint32_t time_step);
  public:
	excentric(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};


#endif