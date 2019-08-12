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

	double get_fuel_max() {return _fuel_max;}
	int set_speed(Complex speed);
	double get_score();
	double get_fuel();
	virtual double get_radius();
	virtual void step() = 0;
	int get_instance();
	int time_step;
	int nb_of_targets;

	virtual double get_relative_distance(int target) = 0;
	virtual double get_relative_delta_speed(int target) = 0;
	virtual Complex get_absolute_position() = 0;
	virtual Complex get_target_absolute_position(int target) = 0;
	virtual Complex get_relative_speed(int target = 0) = 0;
	virtual double get_relative_distance_to_tank() {return 0.;}
	virtual Complex get_tank_absolute_position() {return Complex(0.,0.);}
	virtual double get_tank_fuel() {return 0.;}
	virtual double get_max_tank_fuel() {return 0.;}
	virtual bool is_target_validated(int target) {return false;}
	virtual Complex get_tank_relative_speed() { return Complex(0.,0.);}
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
	double _fuel;
	double _fuel_max;
	double _tank_max;
};

#endif