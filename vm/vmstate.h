#ifndef VM_STATE
#define VM_STATE


#define ADDRESS_SPACE	1<<14

#define D_TYPE		1
#define S_TYPE		2

uint16_t address:14;
uint8_t opcode:4;
enum _inst_type {
  d_type,
  s_type,
};

struct _d_type {
	
	} d_type;
	
struct _s_type {
	
}s_type;


class instruction{
  private:
	enum inst_type type;
	
	opcode _op;
	address _arg1;
	address _arg2;
	
  public:
	instruction(opcode op, address arg1, address arg2);
};



class vm_state {
  private:
	address pc;
	bool status;
	
	static vm_state* _instance;
	double memory[ADDRESS_SPACE];
	instruction code[ADDRESS_SPACE];
	address input_ports[ADDRESS_SPACE];
	address output_ports[ADDRESS_SPACE];
	
  public:
	vm_state();
  
	
	static vm_state *instance() {
	  if (!_instance)
		_instance = new vm_state();
	  return _instance;
	}
}

#endif