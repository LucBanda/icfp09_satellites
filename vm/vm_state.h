#ifndef VM_STATE_H
#define VM_STATE_H
#include "common.h"

//#define GENERATE

class instruction;

#define ADDRESS_RANGE	1<<14
#define instance_port 16000

#define FIRST_READ     0x1
#define FIRST_WRITE    0x2
#define READ           0x4
#define WRITE          0x8
#define ACCESSED       0xB

typedef uint16_t address;


class satellite {
	public:
		satellite() {};
		satellite(double x, double y):pos_x(x), pos_y(y) {};
		public:
		double pos_x;
		double pos_y;
};

class vm_state {
  public:

	vm_state();
	virtual ~vm_state() {};
	void reset();
	double get_pos_x();
	double get_pos_y();
	int set_speed(double vx, double vy);
	double get_score();
	double get_fuel();
	virtual double get_radius();
	virtual void step();
	int get_instance();
	virtual vector<satellite> get_targets();

  protected:
  	address pc;
	bool status;
	double *memory;
	double *input_ports;
	double *output_ports;
	int _instance;
	double old_pos_x;
	double old_pos_y;
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

};

#endif