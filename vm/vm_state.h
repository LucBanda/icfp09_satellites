#ifndef VM_STATE_H
#define VM_STATE_H

#include "common.h"

//#define GENERATE

class instruction;

#define ADDRESS_RANGE	1<<14

typedef uint16_t address;

class vm_state {
  public:
	address pc;
	bool status;
	
	double memory[ADDRESS_RANGE];
	instruction *code[ADDRESS_RANGE];
	double input_ports[ADDRESS_RANGE];
	double output_ports[ADDRESS_RANGE];
	
  
	vm_state(int instance = 0);
	virtual ~vm_state();
	virtual vm_state *clone();
	void load_file(char* file);
	virtual void step();
};

extern vm_state *vm;

#endif