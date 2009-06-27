#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "trace_generator.h"


class Icontroller {
  private:
	trace_generator * _trace;
  public :
	Icontroller(trace_generator* trace) : _trace(trace){};
	virtual bool step (uint32_t time_step) = 0;
};

class hohmann : public Icontroller {
  private:
	
  public:
	hohmann(trace_generator *trace):Icontroller(trace){};
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};
#endif