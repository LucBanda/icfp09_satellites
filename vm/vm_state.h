#ifndef VM_STATE_H
#define VM_STATE_H
#include <complex>
#include "common.h"

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
	virtual void step() = 0;
	int get_instance();
	virtual vector<Complex> get_targets();
	Complex get_speed();
	int time_step;

   protected:
   	void step_state();
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
	double _radius;

   private:
	Complex _old_pos;
	Complex _speed;
	double _fuel;
};

#endif