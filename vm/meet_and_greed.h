#ifndef MEET_AND_GREED_H
#define MEET_AND_GREED_H

#include "common.h"
#include "controller.h"
#include <complex>



class meetandgreed : public Icontroller {
  private:
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	satellite *me;
	satellite *target;
	
	complex<double> calculate_action(uint32_t time_step);
  public:
	meetandgreed(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};


#endif
