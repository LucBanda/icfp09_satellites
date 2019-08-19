#include "agent.h"
#include "bin.h"
#include "complex"

#define ORBIT_PRECISION 1000.
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
	if (vm->time_step > 0) stick_to_target();
	vm->step();
}

void agent::set_execution_map(executionT *map) {
	execution_map = *map;
	int max_time_action = 0;
	for (executionT::iterator it = map->begin(); it != map->end(); ++it) {
		if (it->first > max_time_action) max_time_action = it->first;
	}
	if (max_time_action < 50000)
		max_time_step = max_time_action + 60000;
	else if (max_time_action < 100000)
		max_time_step = max_time_action + 100000;
	else if (max_time_action < 200000)
		max_time_step = max_time_action + 150000;
	else if (max_time_action < 400000)
		max_time_step = max_time_action + 180000;
	else if (max_time_action < 500000)
		max_time_step = max_time_action + 200000;
	else if (max_time_action < 700000)
		max_time_step = max_time_action + 250000;
	else if (max_time_action < 1000000)
		max_time_step = max_time_action + 300000;
	else
		max_time_step = min(2e6, max_time_action + 600000.);
}

double agent::run(int until) {
	int run_until_ts = until;
	if (run_until_ts == 0) run_until_ts = max_time_step;
	while (vm->time_step < run_until_ts) {
		step();
		double score = get_score();
		if (score != 0.0) return score;
	}
	return -1;
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

	if (abs(vm->get_radius() - abs(pos)) < ORBIT_PRECISION &&
		fly_state != FS_STICK_TO_TARGET) {
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
	if (abs(vm->get_absolute_position()) > 2 * vm->get_radius())
		return -1000000;
	if (vm->get_score() == -1)
		return -1000000;
	else if (vm->get_score() != 0) {
		return vm->get_score();
	}
	if (vm->get_fuel() <= 0.001) return -1000000;
	if (vm->time_step > last_time_fs_changed + 1000 &&
		fly_state == FS_ON_TARGET_ORBIT) {
		double approximate_error = log2l(max(1., distance_when_crossed - 500.));
		double fake_score =
			2 * (25. +
				 45. * ((vm->get_fuel() - abs_delta_v_when_crossed) /
						vm->get_fuel_max()) +
				 (30. - log2(vm->time_step / 1000.)) - approximate_error);
		return fake_score;
	}
	return 0;
}

bool agent2::stick_to_target() {
	Complex pos = vm->get_absolute_position();

	double dist = vm->get_relative_distance(0);
	if (vm->time_step == 0) return false;

	if (dist < SAT_PRECISION && fly_state != FS_STICK_TO_TARGET) {
		vm->set_speed(-vm->get_relative_speed(
			0) /* + (vm->get_relative_speed(0) * 0.0001)*/);
		fly_state = FS_STICK_TO_TARGET;
		last_time_fs_changed = vm->time_step;
		return true;
	} else if (abs(vm->get_radius() - abs(pos)) < ORBIT_PRECISION &&
			   fly_state == FS_FLY) {
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
	if (vm->time_step > last_time_fs_changed + 1000 &&
		fly_state == FS_ON_TARGET_ORBIT) {
		double approximate_error = log2l(max(1., distance_when_crossed - 500.));
		double fake_score =
			4 * (25. +
				 45. * ((vm->get_fuel() - abs_delta_v_when_crossed) /
						vm->get_fuel_max()) +
				 (30. - log2(vm->time_step / 1000.)) - approximate_error);
		return fake_score;
	}
	return 0;
}

bool agent3::stick_to_target() {
	if (vm->time_step == 0) return false;
	if (vm->get_relative_distance(0) < SAT_PRECISION &&
		fly_state != FS_STICK_TO_TARGET) {
		vm->set_speed(-vm->get_relative_speed(0));
		fly_state = FS_STICK_TO_TARGET;
		last_time_fs_changed = vm->time_step;
		return true;
	}

	switch (fly_state) {
		case FS_FLY:
			if (abs(abs(vm->get_absolute_position()) -
					abs(vm->get_target_absolute_position(0))) <
				ORBIT_PRECISION) {
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

agent4::agent4(int instance) : agent(instance) {
	max_distance = 0.;
	for (int i = 0; i < vm->nb_of_targets; i++) {
		closest_distance.push_back(9e50);
		closest_time.push_back(0);
		closest_radius.push_back(0);
		non_validated_targets.push_back(i);
		if (max_distance < abs(vm->get_target_absolute_position(i)))
			max_distance = abs(vm->get_target_absolute_position(i));
	}
}

agent4::~agent4() {}

void agent4::set_resume_point(agent *from_agent) {
	vm->set_state(from_agent->vm);
	non_validated_targets = ((agent4 *)from_agent)->non_validated_targets;
	validated_targets = ((agent4 *)from_agent)->validated_targets;
	validated_time_steps = ((agent4 *)from_agent)->validated_time_steps;
	max_distance = ((agent4 *)from_agent)->max_distance;
	for (int i = 0; i < vm->nb_of_targets; i++) {
		closest_distance.push_back(9e50);
		closest_time.push_back(0);
		closest_radius.push_back(0);
	}
}

double agent4::get_intermediate_score() {
	double approximate_error = 0.;
	double fake_score = 0.;
	double best_error = 0;

	for (vector<int>::iterator t1 = validated_time_steps.begin();
		 t1 != validated_time_steps.end(); ++t1)
		fake_score += 1. / 12. - *t1 / 24e6;

	for (vector<int>::iterator tnv = non_validated_targets.begin();
		 tnv != non_validated_targets.end(); ++tnv) {
		double error =
			(2e6 - closest_time[*tnv]) / 24e6 * 75. *
			max(0., min(1., (log2(max_distance) -
							 log2(max(2., closest_distance[*tnv] - 700.))) /
								(1 + log2(max_distance))));

		if (best_error < error && last_validated_time < closest_time[*tnv]) {
			best_error = error;
		}
	}
	approximate_error = best_error;

	fake_score =
		75. * fake_score + 25. * (vm->get_fuel() + vm->get_tank_fuel()) /
							   (vm->get_fuel_max() + vm->get_max_tank_fuel());
	fake_score += approximate_error;
	fake_score *= 8.;
	return fake_score;
}

double agent4::get_score() {
	if (vm->time_step < 2) return 0;
	if (vm->time_step == max_time_step) return get_intermediate_score();
	if (fly_state == FS_LOST) return -1;
	if (vm->get_fuel() <= 0.001) return -1;
	if (vm->get_score() != 0) return vm->get_score();

	return 0;
}

void agent4::update_status() {
	if (vm->time_step == 1) {
	}

	vector<int>::iterator target = non_validated_targets.begin();
	while (target != non_validated_targets.end()) {
		if (vm->is_target_validated(*target)) {
			validated_targets.push_back(*target);
			target = non_validated_targets.erase(target);
			validated_time_steps.push_back(vm->time_step);
			fly_state = FS_FLY;
			last_validated_time = vm->time_step;
			last_time_fs_changed = vm->time_step;
		} else {
			target++;
		}
	}
}

bool agent4::stick_to_target() {
	update_status();

	for (vector<int>::iterator target = non_validated_targets.begin();
		 target != non_validated_targets.end(); ++target) {
		if (closest_distance[*target] > vm->get_relative_distance(*target)) {
			int tg = *target;
			closest_distance[tg] = vm->get_relative_distance(tg);
			closest_radius[tg] = abs(vm->get_target_absolute_position(tg));
			closest_time[tg] = vm->time_step;
		}
	}
	return false;
}
