#ifndef CLEAR_SKY_H
#define CLEAR_SKY_H

#include "common.h"
#include "controller.h"
#include <complex>



class clear_sky : public Icontroller {
  private:
	address _delta_vx_addr;
	address _delta_vy_addr;
	
	satellite *me;
	satellite *target[11];
	satellite *fuelling;
	
	complex<double> calculate_action(uint32_t time_step);
	int first_not_checked;
	  
  public:
	clear_sky(trace_generator *trace, double instance);
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};


#endif
