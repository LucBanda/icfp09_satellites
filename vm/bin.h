#ifndef BIN_H
#define BIN_H
#include "vm_state.h"

class bin_1 :public vm_state {
  public:
	void step();
	vm_state *clone() {
	    bin_1 *cloned = new bin_1;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * ADDRESS_RANGE);  
		memcpy(cloned->code, 		code,		sizeof(instruction*) * ADDRESS_RANGE);
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};

class bin_2 :public vm_state {
  public:
	void step();
		vm_state *clone() {
	    bin_2 *cloned = new bin_2;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * ADDRESS_RANGE);  
		memcpy(cloned->code, 		code,		sizeof(instruction*) * ADDRESS_RANGE);
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};

class bin_3 :public vm_state {
  public:
	void step();
		vm_state *clone() {
	    bin_3 *cloned = new bin_3;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * ADDRESS_RANGE);  
		memcpy(cloned->code, 		code,		sizeof(instruction*) * ADDRESS_RANGE);
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};

class bin_4 :public vm_state {
  public:
	void step();
		vm_state *clone() {
	    bin_4 *cloned = new bin_4;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * ADDRESS_RANGE);
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * ADDRESS_RANGE);  
		memcpy(cloned->code, 		code,		sizeof(instruction*) * ADDRESS_RANGE);
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};


#endif