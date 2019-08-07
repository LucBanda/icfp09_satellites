#include "agent.h"
#include "bin.h"
#include "complex"

#define ORBIT_PRECISION 750.
#define SAT_PRECISION 750.

agent::agent(int instance) {
	vm = bin_factory(instance);
	last_time_fs_changed = 0;
	fly_state = FS_ORBIT;
}

agent::~agent() { delete vm; }

void agent::step() {
	executionT::iterator it = execution_map.find(vm->time_step);
	if (it != execution_map.end()) {
		vm->set_speed(it->second);
		fly_state = FS_FLY;
	} else {
		vm->set_speed(Complex(0., 0.));
	}
	if (fly_state != FS_ORBIT)
		stick_to_target();

	vm->step();
}

void agent::set_execution_map(executionT *map) { execution_map = *map; }

double agent::run() {
	while (vm->time_step < max_time_step) {
		step();
		double score = get_score();
		if (score != 0.0)
			return score;
	}
	return -1000000;
}

agent1::agent1(int instance) : agent(instance) {}

agent1::~agent1() {}

double agent1::get_score() {
	if (abs(vm->get_absolute_position()) > 2 * vm->get_radius()) return -1;
	double score = vm->get_score();
	if (score == -1) {
		return score;
	} else if (score != 0) {
		return score;
	}
	if (vm->get_fuel() <= 0.001) return -1;
	return 0;
}

bool agent1::stick_to_target() {
	Complex pos = vm->get_absolute_position();

	if (abs(vm->get_radius() - abs(pos)) < ORBIT_PRECISION && fly_state != FS_STICK_TO_TARGET) {
		Complex speed = vm->get_relative_speed();
		double abs_v_dest = sqrt(MU / abs(pos));
		double arg_v_dest_p = arg(pos) + M_PI / 2.;
		double arg_v_dest_m = arg(pos) - M_PI / 2.;
		Complex delta_v_dest_p = polar(abs_v_dest, arg_v_dest_p) - speed;
		Complex delta_v_dest_m = polar(abs_v_dest, arg_v_dest_m) - speed;
		if (abs(delta_v_dest_p) > abs(delta_v_dest_m)) {
			vm->set_speed(delta_v_dest_m);
		} else {
			vm->set_speed(delta_v_dest_p);
		}
		fly_state = FS_STICK_TO_TARGET;
		return true;
	}
	return false;
}

agent2::agent2(int instance) : agent(instance) {}

agent2::~agent2() {}

double agent2::get_score() {
	if (abs(vm->get_absolute_position()) > 2 * vm->get_radius()) return -1000000;
		if (vm->get_score() == -1)
			return -1000000;
		else if (vm->get_score() != 0) {
			return vm->get_score();
	}
	if (vm->get_fuel() <= 0.001) return -1000000;
	if (vm->time_step > last_time_fs_changed + 1000 && fly_state == FS_ON_TARGET_ORBIT) {
		double approximate_error = log2f(max(1., distance_when_crossed - 500.));
		double fake_score = 2 * (25. + 45. * ((vm->get_fuel() - abs_delta_v_when_crossed) / vm->get_fuel_max()) + (30. - log2(vm->time_step/1000.)) - approximate_error);
		return fake_score;
	}
	return 0;
}

bool agent2::stick_to_target() {
	Complex pos = vm->get_absolute_position();

	double dist  = vm->get_relative_distance(0);
	if (vm->time_step == 0)
		return false;

	if (dist < SAT_PRECISION && fly_state != FS_STICK_TO_TARGET) {
		vm->set_speed(-vm->get_relative_speed(0)/* + (vm->get_relative_speed(0) * 0.0001)*/);
		fly_state = FS_STICK_TO_TARGET;
		last_time_fs_changed = vm->time_step;
		return true;
	} else if (abs(vm->get_radius() - abs(pos)) < ORBIT_PRECISION && fly_state == FS_FLY) {
		fly_state = FS_ON_TARGET_ORBIT;
		last_time_fs_changed = vm->time_step;
		distance_when_crossed = dist;
		abs_delta_v_when_crossed = vm->get_relative_delta_speed(0);
		return true;
	}
	return false;
}

agent3::agent3(int instance) : agent(instance) {}

agent3::~agent3() {}

double agent3::get_score() {
	if (vm->get_score() == -1) {
		return -10000;
	} else if (vm->get_score() != 0)
		return vm->get_score();

	if (vm->get_fuel() <= 0.001) return -10000;
	if (vm->time_step > last_time_fs_changed + 1000 && fly_state == FS_ON_TARGET_ORBIT) {
		double approximate_error = log2f(max(1., distance_when_crossed - 500.));
		double fake_score = 4 * (25. + 45. * ((vm->get_fuel() - abs_delta_v_when_crossed) / vm->get_fuel_max()) + (30. - log2(vm->time_step/1000.)) - approximate_error);
		return fake_score;
	}
	return 0;
}

bool agent3::stick_to_target() {
	if (vm->time_step == 0)
		return false;
	if (vm->get_relative_distance(0) <  SAT_PRECISION  && fly_state != FS_STICK_TO_TARGET) {
		vm->set_speed(-vm->get_relative_speed(0));
		fly_state = FS_STICK_TO_TARGET;
		last_time_fs_changed = vm->time_step;
		return true;
	}

	switch (fly_state) {
	case FS_FLY:
		if (abs(abs(vm->get_absolute_position()) - abs(vm->get_target_absolute_position(0))) < ORBIT_PRECISION) {
			fly_state = FS_ON_TARGET_ORBIT;
			last_time_fs_changed = vm->time_step;
			distance_when_crossed = vm->get_relative_distance(0);
			abs_delta_v_when_crossed = vm->get_relative_delta_speed(0);
		}
		break;
	default:
		break;
	}
	return false;
}
