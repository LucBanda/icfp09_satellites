#ifndef BIN_H
#define BIN_H
#include "vm_state.h"

class bin_1 :public vm_state {
  public:
	bin_1();
	void step();
	vm_state *clone() {
	    bin_1 *cloned = new bin_1;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * (max_global-min_global+1));
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * (max_out_port-min_out_port+1));
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * 4);   
		cloned->max_out_port = max_out_port;
		cloned->min_out_port = min_out_port;
		cloned->max_global = max_global;
		cloned->min_global = min_global;
		
		cloned->status = status;
		cloned->pc = pc;
		return cloned;
	  
	};
};

class bin_2 :public vm_state {
  public:
	bin_2();
	void step();
	vm_state *clone() {
	    bin_2 *cloned = new bin_2;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * (max_global-min_global+1));
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * (max_out_port-min_out_port+1));
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * 4);   
		cloned->max_out_port = max_out_port;
		cloned->min_out_port = min_out_port;
		cloned->max_global = max_global;
		cloned->min_global = min_global;
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};

class bin_3 :public vm_state {
  public:
	bin_3();
	void step();
	vm_state *clone() {
	    bin_3 *cloned = new bin_3;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * (max_global-min_global+1));
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * (max_out_port-min_out_port+1));
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * 4);  
		cloned->max_out_port = max_out_port;
		cloned->min_out_port = min_out_port;
		cloned->max_global = max_global;
		cloned->min_global = min_global;
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};

class bin_4 :public vm_state {
  public:
	bin_4();
	void step();
	vm_state *clone() {
	    bin_4 *cloned = new bin_4;
		
		memcpy(cloned->memory,	memory, 	sizeof(double) * (max_global-min_global+1));
		memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * (max_out_port-min_out_port+1));
		memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * 4);  
		cloned->max_out_port = max_out_port;
		cloned->min_out_port = min_out_port;
		cloned->max_global = max_global;
		cloned->min_global = min_global;
		cloned->status = status;
		cloned->pc = pc;
		
		return cloned;
	  
	};
};


#endif