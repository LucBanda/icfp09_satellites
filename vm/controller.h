#ifndef CONTROLLER_H
#define CONTROLLER_H

class Icontroller {
  private:
	trace_generator * _trace;
  public :
	Icontroller(trace_generator* trace) : _trace(trace){};
	virtual void step (uint32_t time_step) = 0;
}

class hohmann : public controller {
  private:
	
  public:
	hohmann(trace_generator *trace):Icontroller(trace){};
	virtual bool step (uint32_t time_step);
	virtual void monitor();
};
#endif