#include "vm_state.h"
#include "common.h"



vm_state::vm_state(): _radius(0.) {
	time_step = 0;
	_fuel = 0;
}

void vm_state::reset() {
	memset(output_ports, 0, sizeof(double) * (max_out_port - min_out_port + 1));
	memset(input_ports, 0, sizeof(double) * 4);
	memset(memory, 0, sizeof(double) * (max_global - min_global + 1));
	input_ports[instance_addr] = _instance;
	status = 0;
	pc = 0;
	time_step = 0;
}

int vm_state::set_speed(Complex loc_speed) {
	input_ports[delta_vx_addr] = real(loc_speed);
	input_ports[delta_vy_addr] = imag(loc_speed);
	return 0;
}
int vm_state::get_instance() { return input_ports[instance_addr]; }

double vm_state::get_fuel() { return _fuel; }

double vm_state::get_score() { return output_ports[score_addr]; }

double vm_state::get_radius() { return _radius; }
