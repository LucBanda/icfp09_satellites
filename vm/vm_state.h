#ifndef VM_STATE
#define VM_STATE

#include "common.h"

#define ADDRESS_SPACE	1<<14

typedef uint16_t address;

class instruction{
  
  public:
	static instruction *parse(uint32_t raw);
	virtual double execute() =0;
};

class d_type : public instruction {
  private:
	double _arg1;
	double _arg2;
  public:
	static d_type* parse(uint32_t raw);
	virtual double execute() = 0;
	
};

class s_type : public instruction {
  private:
  double _arg1;
  public:
	static s_type* parse(uint32_t raw);
	virtual double execute() = 0;
};

class noop {
  public:
	virtual double execute();
};





class vm_state {
  private:
	address pc;
	bool status;
	
	static vm_state* _instance;
	double memory[ADDRESS_SPACE];
	instruction *code[ADDRESS_SPACE];
	address input_ports[ADDRESS_SPACE];
	address output_ports[ADDRESS_SPACE];
	
  public:
	vm_state();
  
	
	static vm_state *instance() {
	  if (!_instance)
		_instance = new vm_state();
	  return _instance;
	}
};

#endif