#include "agent.h"
#include "bin.h"
#include "complex"

#define SAT_PRECISION 200.

agent::agent(int instance) {
	vm = bin_factory(instance);
	finalized = false;
}

agent::~agent() { delete vm; }

void agent::step() {
	executionT::iterator it = execution_map.find(vm->time_step);
	if (it != execution_map.end()) {
		vm->set_speed(it->second);
	} else {
		vm->set_speed(Complex(0, 0));
	}
	if (!finalized)
		finalized = stick_to_target();

	vm->step();
}

void agent::set_execution_map(executionT *map) { execution_map = *map; }

double agent::run() {
	while (vm->time_step < max_time_step) {
		step();
		if (abs(vm->get_pos()) > 2 * vm->get_radius()) return -1.;
		if (vm->get_score() != 0) return vm->get_score();
		if (vm->get_fuel() <= 0) return -1.;
	}

	return 0.;
}


agent1::agent1(int instance):agent(instance) {}

agent1::~agent1() {}

bool agent1::stick_to_target() {
	Complex pos = vm->get_pos();
	Complex speed = vm->get_speed();

	if (abs(vm->get_radius() - abs(pos)) < SAT_PRECISION) {
		double abs_v_dest = sqrt(MU / abs(pos));
		double arg_v_dest_p = arg(pos) + M_PI / 2;
		double arg_v_dest_m = arg(pos) - M_PI / 2;
		Complex delta_v_dest_p = polar(abs_v_dest, arg_v_dest_p) - speed;
		Complex delta_v_dest_m = polar(abs_v_dest, arg_v_dest_m) - speed;
		if (abs(delta_v_dest_p) > abs(delta_v_dest_m)) {
			vm->set_speed(delta_v_dest_m);
		} else {
			vm->set_speed(delta_v_dest_p);
		}
		return true;
	}
	return false;
}

agent2::agent2(int instance):agent(instance) {}

agent2::~agent2() {}

bool agent2::stick_to_target() {

	return false;
}