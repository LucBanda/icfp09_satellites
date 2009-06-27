#ifndef VM_STATE_H
#define VM_STATE_H

#include "common.h"

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
	
  
	vm_state();
  
	
/*	static vm_state *instance() {
	  if (!vm_state::_instance)
		vm_state::_instance = new vm_state();
	  return vm_state::_instance;
	}
	*/
	void load_file(char* file);
	void step();
};

extern vm_state *vm;

#endif