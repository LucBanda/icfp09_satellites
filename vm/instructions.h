#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include "vm_state.h"

class instruction{
  public:
	static instruction *parse(uint32_t raw);
	virtual void execute(vm_state *cur_vm) =0;
};

class d_type : public instruction {
  protected:
	address _arg1;
	address _arg2;
  public:
	d_type(address arg1, address arg2) : instruction(), _arg1(arg1), _arg2(arg2){};
	static d_type* parse(uint32_t raw);
	virtual void execute(vm_state *cur_vm) = 0;
	
};

class s_type : public instruction {
  protected:
  address _arg1;
  public:
	s_type(address arg1) : instruction(), _arg1(arg1) {};
	static s_type* parse(uint32_t raw);
	virtual void execute(vm_state *cur_vm) = 0;
};

class add :public d_type {
  public:
	add(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class sub :public d_type {
  public:
	sub(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class mult :public d_type {
  public:
	mult(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class vdiv :public d_type {
  public:
	vdiv(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class output :public d_type {
  public:
	output(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class phi :public d_type {
  public:
	phi(address arg1, address arg2) : d_type(arg1, arg2){};
	virtual void execute(vm_state *cur_vm);
};

class noop :public s_type{
  public:
	noop(address arg1) : s_type(arg1){};
	virtual void execute(vm_state *cur_vm);
};

enum cmp_type {
  LTZ,
  LEZ,
  EQZ,
  GEZ,
  GTZ,
};

class cmpz :public s_type{
  private:
	uint8_t _immediate;
  public:
	cmpz(address arg1, uint8_t immediate) : s_type(arg1), _immediate(immediate & 0x7) {};
	virtual void execute(vm_state *cur_vm);
};

class vsqrt :public s_type{
  public:
	vsqrt(address arg1) : s_type(arg1){};
	virtual void execute(vm_state *cur_vm);
};

class vcopy :public s_type{
  public:
	vcopy(address arg1) : s_type(arg1){};
	virtual void execute(vm_state *cur_vm);
};

class input :public s_type{
  public:
	input(address arg1) : s_type(arg1){};
	virtual void execute(vm_state *cur_vm);
};


#endif