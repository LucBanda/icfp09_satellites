#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include "vm_state.h"

class instruction{
  
  public:
	static instruction *parse(uint32_t raw);
	virtual double execute() =0;
};

class d_type : public instruction {
  private:
	double *_arg1;
	double *_arg2;
  public:
	d_type(double *arg1, double *arg2) : instruction(), _arg1(arg1), _arg2(arg2){};
	static d_type* parse(uint32_t raw);
	virtual double execute() = 0;
	
};

class s_type : public instruction {
  private:
  double *_arg1;
  public:
	s_type(double *arg1) : instruction(), _arg1(arg1) {};
	static s_type* parse(uint32_t raw);
	virtual double execute() = 0;
};

class add :public d_type {
  public:
	add(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class sub :public d_type {
  public:
	sub(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class mult :public d_type {
  public:
	mult(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class div :public d_type {
  public:
	div(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class output :public d_type {
  public:
	output(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class phi :public d_type {
  public:
	phi(double *arg1, double *arg2) : d_type(arg1, arg2){};
	virtual double execute();
};

class noop :public s_type{
  public:
	noop(double *arg1) : s_type(arg1){};
	virtual double execute();
};

enum cmp_type {
  LTZ,
  LEZ,
  EQZ,
  GEZ,
  GTZ,
};

class cmpz :public s_type{
  enum cmp_type _immediate;
  public:
	cmpz(double *arg1, enum cmp_type immediate) : s_type(arg1), _immediate(immediate) {};
	virtual double execute();
};

class sqrt :public s_type{
  public:
	sqrt(double *arg1) : s_type(arg1){};
	virtual double execute();
};

class copy :public s_type{
  public:
	copy(double *arg1) : s_type(arg1){};
	virtual double execute();
};

class input :public s_type{
  public:
	input(double *arg1) : s_type(arg1){};
	virtual double execute();
};


#endif