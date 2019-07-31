#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "vm_state.h"

typedef std::map<int, std::tuple<double, double>> executionT;

class agent {
   public:
	vm_state *vm;
	double max_time_step = 10e6;

	agent(int instance);
	~agent();

	void apply_proportionnal_speed(double speed, double alpha);
	double run(executionT *execution_map);
};
#endif