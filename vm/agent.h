#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "vm_state.h"

typedef std::map<int, Complex> executionT;


enum flying_state {
	FS_ORBIT,
	FS_FLY,
	FS_DECELERATE,
};

class agent {
   public:
	vm_state *vm;
	double max_time_step = 10e6;

	agent(int instance);
	~agent();

	bool circular_orbit_now();
	void step(int time_step);
	void set_execution_map(executionT *map);
	void set_speed(Complex speed);
	double run();

   private:
	executionT execution_map;
	bool finalized;
	enum flying_state fs_state;
};
#endif