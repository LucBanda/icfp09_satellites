#include "agent.h"
#include "bin.h"
#include "complex"

#define SAT_PRECISION 1000

agent::agent(int instance) {
	vm = bin_factory(instance);
	finalized = false;
	fs_state = FS_ORBIT;
}

agent::~agent() { delete vm; }

void agent::set_speed(Complex speed) {
	Complex current_speed = vm->get_speed();

	Complex delta_v =
		polar(abs(speed), arg(speed) + arg(current_speed));
	// cout << "speed " << delta_v << endl;
	vm->set_speed(delta_v);
}

bool agent::circular_orbit_now() {
	Complex pos = vm->get_pos();
	Complex speed = vm->get_speed();

	if (abs(vm->get_radius() - abs(pos)) < SAT_PRECISION &&
		((fs_state == FS_FLY) || (fs_state == FS_DECELERATE))) {
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
		fs_state = FS_ORBIT;
		// cout << "relative pos(true) " << vm->get_radius() - abs(pos) << endl;
		return true;
	} else if ((abs(vm->get_radius() - abs(pos)) <
				2 * abs(vm->get_speed()) + SAT_PRECISION) &&
			   fs_state == FS_FLY) {
		Complex delta_v_dest = polar(500., arg(speed)) - speed;
		vm->set_speed(delta_v_dest);
		fs_state = FS_DECELERATE;
	}
	return false;
}

void agent::step(int time_step) {
	executionT::iterator it = execution_map.find(time_step);
	if (it != execution_map.end()) {
		fs_state = FS_FLY;
		set_speed(it->second);
	} else {
		vm->set_speed(Complex(0, 0));
	}
	if (time_step > 1 && !finalized) finalized = circular_orbit_now();

	vm->step();
}

void agent::set_execution_map(executionT *map) { execution_map = *map; }

double agent::run() {
	int i = 0;
	while (i < max_time_step) {
		step(i);
		if ((i > 2) && (abs(vm->get_pos()) > 2 * vm->get_radius())) return -1;
		if (vm->get_score() != 0) return vm->get_score();
		if (vm->get_fuel() <= 0) return -1;
		i++;
	}

	return 0;
}