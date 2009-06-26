#include "instructions.h"
#include "math.h"

instruction *instruction::parse(uint32_t raw)
{
  int opcode = raw>>28;
  if (opcode == 0)
	return s_type::parse(raw);
  else
	return d_type::parse(raw);
}

s_type *s_type::parse(uint32_t raw) 
{
  int zero = raw >> 28;
  int opcode = (raw >> 24) & 0xf;
  address address1 = raw & 0x1fff;
  
  assert(zero == 0);
  
  switch (opcode) {
	case 0: return new noop(address1);
	case 1: return new cmpz(address1, (raw >> 20)&0xf);
	case 2: return new vsqrt(address1);
	case 3: return new vcopy(address1);
	case 4: return new input(address1);
	default: cerr << "opcode " << opcode << "not recognized in s_syle creation with raw = "<< hex << raw<< endl;
  }
  return NULL;
}

d_type *d_type::parse(uint32_t raw) 
{
  int opcode = (raw >> 28) & 0xf;
  address address1 = (raw>>14) & 0x1fff;
  address address2 = raw & 0x1fff;
  
  switch (opcode) {
	case 1: return new add(address1, address2);
	case 2: return new sub(address1, address2);
	case 3: return new mult(address1, address2);
	case 4: return new div(address1, address2);
	case 5: return new output(address1, address2);
	case 6: return new phi(address1, address2);
	default: cerr << "opcode " << opcode << "not recognized in d_syle creation with raw = "<< hex << raw<< endl;
  }
  return NULL;
}

void add::execute(){
  cerr << vm->memory[_arg1] << " + " << vm->memory[_arg2] << endl;
  vm->memory[vm->pc] = vm->memory[_arg1] + vm->memory[_arg2];
  
}

void sub::execute(){
  cerr << vm->memory[_arg1] << " - " << vm->memory[_arg2] << endl;
  vm->memory[vm->pc] = vm->memory[_arg1] - vm->memory[_arg2];
}

void mult::execute(){
  cerr << vm->memory[_arg1] << " * " << vm->memory[_arg2] << endl;
  vm->memory[vm->pc] = vm->memory[_arg1] * vm->memory[_arg2];
}

void div::execute(){
  cerr << vm->memory[_arg1] << " / " << vm->memory[_arg2] << endl;
  if (vm->memory[_arg2] == 0)
	vm->memory[vm->pc] = 0;
  else
	vm->memory[vm->pc] = vm->memory[_arg1] / vm->memory[_arg2];
}

void output::execute() {
	cerr << "port "<< _arg1 << " = " << vm->memory[_arg2] << endl;
    vm->output_ports[_arg1] = vm->memory[_arg2];
}

void phi::execute() {
  cerr << "phi " << vm->status << (double)(vm->status ?  vm->memory[_arg1] : vm->memory[_arg2] )<< endl;
  if (vm->status)
	vm->memory[vm->pc] = vm->memory[_arg1];
  else
	vm->memory[vm->pc] = vm->memory[_arg2];
}

void noop::execute() {
  //cerr << "noop" << endl;
	vm->memory[vm->pc] = vm->memory[vm->pc];
}

void cmpz::execute() {
  cerr << "cmpz" << endl;
	bool result;
	double val = vm->memory[_arg1];
	switch (_immediate) {
	  case 0: result = (val < 0.0); break;
	  case 1: result = (val <= 0.0); break;
	  case 2: result = (val = 0.0); break;
	  case 3: result = (val >= 0.0); break;
	  case 4: result = (val > 0.0); break;
	  default: cerr<<"unknown immediate address in cmpz : " << _immediate << endl;
	}
	vm->status = result;
}

void vsqrt::execute() {
  cerr << "sqrt " << vm->memory[_arg1] << endl;
  vm->memory[vm->pc] = sqrt(vm->memory[_arg1]);
}

void vcopy::execute() {
  cerr << "copy " << vm->memory[_arg1] << endl;
  vm->memory[vm->pc] = vm->memory[_arg1];
}

void input::execute() {
  cerr << "load port " << _arg1<< endl;
  vm->memory[vm->pc] = vm->input_ports[_arg1];
}