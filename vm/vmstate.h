#ifndef VM_STATE
#define VM_STATE


#define ADDRESS_SPACE	1<<14

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