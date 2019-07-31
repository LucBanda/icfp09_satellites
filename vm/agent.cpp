#include "agent.h"
#include "bin.h"
#include "complex"

agent::agent(int instance) { vm = bin_factory(instance); }
agent::~agent() { delete vm; }

void agent::apply_proportionnal_speed(double speed, double alpha) {
	Complex current_speed = vm->get_speed();

	if (speed < 0) {
		alpha += M_PI;
		speed = -speed;
	}
	Complex delta_v =
		polar(abs(current_speed) * speed, alpha + arg(current_speed));
	vm->set_speed(delta_v);
}

double agent::run(executionT *execution_map) {
	int i = 0;
    double score = 0;

	for (executionT::iterator it = execution_map->begin(); it != execution_map->end(); it++) {
		if (it->first > 10)
			max_time_step = it->first * 3;
	}
	while (i < max_time_step) {
		executionT::iterator it = execution_map->find(i);
		if (it != execution_map->end()) {
			apply_proportionnal_speed(std::get<0>(it->second),
									  std::get<1>(it->second));
		} else
			vm->set_speed(Complex(0,0));
		vm->step();
		if (vm->get_score() != 0) return vm->get_score();
		if (vm->get_fuel() <= 0)
			return - 1;
		i++;
	}

	return 0;
}