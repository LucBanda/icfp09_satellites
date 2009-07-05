#include "instructions.h"

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
  address address1 = raw & 0x3fff;
  
  assert(zero == 0);
  
  switch (opcode) {
	case 0: return new noop(address1);
	case 1: return new cmpz(address1, (raw >> 21)&0x7);
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
  address address1 = (raw>>14) & 0x3fff;
  address address2 = raw & 0x3fff;
  
  switch (opcode) {
	case 1: return new add(address1, address2);
	case 2: return new sub(address1, address2);
	case 3: return new mult(address1, address2);
	case 4: return new vdiv(address1, address2);
	case 5: return new output(address1, address2);
	case 6: return new phi(address1, address2);
	default: cerr << "opcode " << opcode << "not recognized in d_syle creation with raw = "<< hex << raw<< endl;
  }
  return NULL;
}

void add::execute(vm_state *cur_vm){
  //cerr << vm->memory[_arg1] << " + " << vm->memory[_arg2] << endl;
  cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1] + cur_vm->memory[_arg2];
  
}

void sub::execute(vm_state *cur_vm){
  //cerr << vm->memory[_arg1] << " - " << vm->memory[_arg2] << endl;
  cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1] - cur_vm->memory[_arg2];
}

void mult::execute(vm_state *cur_vm){
  //cerr << vm->memory[_arg1] << " * " << vm->memory[_arg2] << endl;
  cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1] * cur_vm->memory[_arg2];
}

void vdiv::execute(vm_state *cur_vm){
  //cerr << vm->memory[_arg1] << " / " << vm->memory[_arg2] << endl;
  if (cur_vm->memory[_arg2] == 0)
	cur_vm->memory[cur_vm->pc] = 0;
  else
	cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1] / cur_vm->memory[_arg2];
}

void output::execute(vm_state *cur_vm) {
	//cerr << "port "<< _arg1 << " = " << vm->memory[_arg2] << endl;
    cur_vm->output_ports[_arg1] = cur_vm->memory[_arg2];
}

void phi::execute(vm_state *cur_vm) {
  //cerr << "phi " << vm->status <<  " " << (double)(vm->status ?  vm->memory[_arg1] : vm->memory[_arg2] )<< endl;
  if (cur_vm->status)
	cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1];
  else
	cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg2];
}

void noop::execute(vm_state *cur_vm) {
	cur_vm->memory[cur_vm->pc] = cur_vm->memory[cur_vm->pc];
}

void cmpz::execute(vm_state *cur_vm) {
  //cerr << "cmpz " ;
	bool result;
	double val = cur_vm->memory[_arg1];
	switch (_immediate) {
	  case 0: result = (val < 0.0); /*cerr << "<";*/ break;
	  case 1: result = (val <= 0.0); /*cerr << "<=";*/ break;
	  case 2: result = (val == 0.0); /*cerr << "=";*/ break;
	  case 3: result = (val >= 0.0); /*cerr << ">=";*/ break;
	  case 4: result = (val > 0.0); /*cerr << ">";*/ break;
	  default: result =0; cerr<<"unknown immediate address in cmpz : " << _immediate << endl;
	}
	cur_vm->status = result;
}

void vsqrt::execute(vm_state *cur_vm) {
  //cerr << "sqrt " << vm->memory[_arg1] << endl;
  assert (cur_vm->memory[_arg1] >= 0);
  cur_vm->memory[cur_vm->pc] = sqrt(cur_vm->memory[_arg1]);
}

void vcopy::execute(vm_state *cur_vm) {
  //cerr << "copy " << vm->memory[_arg1] << endl;
  cur_vm->memory[cur_vm->pc] = cur_vm->memory[_arg1];
}

void input::execute(vm_state *cur_vm) {
  //cerr << "load port " << _arg1<< endl;
  cur_vm->memory[cur_vm->pc] = cur_vm->input_ports[_arg1];
}