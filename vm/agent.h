#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "vm_state.h"

typedef std::map<int, Complex> executionT;


class agent {
   public:
	vm_state *vm;
	double max_time_step = 2000000;

	agent(int instance);
	virtual ~agent();

	virtual bool stick_to_target() = 0;
	virtual void step();
	virtual void set_execution_map(executionT *map);
	virtual double run();

   protected:
	executionT execution_map;
	bool finalized;
};

class agent1 : public agent {
   public:
	agent1(int instance);
	~agent1();

	bool stick_to_target();

};

class agent2 : public agent {
   public:
	agent2(int instance);
	~agent2();

	bool stick_to_target();

};
#endif