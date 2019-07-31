#ifndef VM_STATE_H
#define VM_STATE_H
#include <complex>
#include "common.h"

//#define GENERATE

class instruction;

#define ADDRESS_RANGE 1 << 14
#define instance_port 16000

#define FIRST_READ 0x1
#define FIRST_WRITE 0x2
#define READ 0x4
#define WRITE 0x8
#define ACCESSED 0xB

typedef uint16_t address;

class vm_state {
   public:
	vm_state();
	virtual ~vm_state(){};
	void reset();
	Complex get_pos();
	int set_speed(Complex speed);
	double get_score();
	double get_fuel();
	virtual double get_radius();
	virtual void step();
	int get_instance();
	virtual vector<Complex> get_targets();
	Complex get_speed();

   protected:
	address pc;
	bool status;
	double *memory;
	double *input_ports;
	double *output_ports;
	int _instance;
	address min_global;
	address max_global;
	address min_out_port;
	address max_out_port;
	address radius_addr;
	address delta_vx_addr;
	address delta_vy_addr;
	address instance_addr;
	address score_addr;
	address fuel_addr;
	address addr_x;
	address addr_y;
	address pos_x_addr;
	address pos_y_addr;

   private:
	Complex _old_pos;
	Complex _speed;
};

#endif