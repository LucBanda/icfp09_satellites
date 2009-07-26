#ifndef VM_STATE_H
#define VM_STATE_H

#include "common.h"

//#define GENERATE

class instruction;

#define ADDRESS_RANGE	1<<14
#define instance_port 16000

#define FIRST_READ     0x1
#define FIRST_WRITE    0x2
#define READ           0x4
#define WRITE          0x8
#define ACCESSED       0xB

typedef uint16_t address;

class vm_state {
  public:

	address min_global;
	address max_global;
	address min_out_port;
	address max_out_port;

	double input_ports_0;
	double input_ports_1;
	double input_ports_16000;
	
	int state[ADDRESS_RANGE];
	

	
	int pass;
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