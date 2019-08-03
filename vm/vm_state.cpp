#include "vm_state.h"
#include "common.h"

vm_state::vm_state(): _radius(0.), _speed(Complex(0,0)) {
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
	_old_pos = Complex(0, 0);
	time_step = 0;
	_old_targets.clear();
}

void vm_state::step_state() {
	if (_old_pos != Complex(0.0, 0.0)) {
		_speed = get_pos() - _old_pos;
	}
	_old_pos = get_pos();
	_fuel = output_ports[fuel_addr];
	time_step++;
}

int vm_state::set_speed(Complex speed) {
	input_ports[delta_vx_addr] = real(speed);
	input_ports[delta_vy_addr] = imag(speed);

	return 0;
}

Complex vm_state::get_speed() { return _speed; }

int vm_state::get_instance() { return input_ports[instance_addr]; }

double vm_state::get_fuel() { return _fuel; }

double vm_state::get_score() { return output_ports[score_addr]; }

Complex vm_state::get_pos() {
	Complex pos(output_ports[pos_x_addr], output_ports[pos_y_addr]);
	return -pos;
}

double vm_state::get_radius() { return _radius; }

vector<Complex> vm_state::get_targets() {
	return _old_targets;
}

vector<Complex> vm_state::get_targets_speeds() {
	return _speed_targets;
}