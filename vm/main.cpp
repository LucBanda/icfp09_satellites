#include "agent.h"
#include "bin.h"
#include "common.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"

struct main_status {
	vm_state* vm;
	renderer* render;
	agent* ag;
};

bool idle(void* user_param) {
	struct main_status* status = (struct main_status*)user_param;
	vm_state* vm = status->ag->vm;
	agent* ag = status->ag;
	renderer* render = status->render;
	double fuel;

	ag->step();
	render->main_sat = vm->get_pos();
	fuel = vm->get_fuel();
	render->set_fuel(fuel);
	render->set_sat(vm->get_targets());
	if (vm->get_score()) {
		cout << "score = " << vm->get_score() << " fuel = " << vm->get_fuel() << endl;
		return true;
	}
	return false;
}

int main(int argc, char** argv) {
	bool do_all;
	vm_state* vm = NULL;
	struct main_status status;
	renderer* render;

	if (argc < 2) {
		cerr << "please enter the scenario id" << endl;
		exit(-1);
	}
	uint32_t instance = 1001;

	if (strcmp(argv[1], "all") != 0) {
		instance = atoi(argv[1]);
		do_all = false;
	} else
		do_all = true;

	for (int i = 1; i < 5; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				instance = i * 1000 + j;
			}

			agent* ag;
			render = new renderer();
			if (instance / 1000 == 1) {
				ag = new agent1(instance);
			} else if (instance / 1000 == 2) {
				ag = new agent2(instance);
			} else if (instance / 1000 == 3) {
				ag = new agent2(instance);
			} else {
				ag = new agent2(instance);
			}

			vm = ag->vm;
			executionT map;
			if (instance == 1001) {
				map[0] = Complex(1649.156263, -2800.234537); map[7405] = Complex(5077.028557, -1051.155736);
			} else if (instance == 1002) {
				map[0] = Complex(4087.196340, -2819.835164); map[8034] = Complex(2118.740268, 1384.343145);
			} else if (instance == 1003) {
				map[0] = Complex(6308.011994, 501.171667); map[13241] = Complex(-2040.768846, -232.276828);
			} else if (instance == 1004) {
				map[0] = Complex(-2139.581411, 3963.342124); map[7761] = Complex(3010.028293, 1142.908135);
			}

			ag->set_execution_map(&map);
			render->add_radius(vm->get_radius());
			render->set_max_fuel(vm->get_fuel());

			status.vm = vm;
			status.render = render;
			status.ag = ag;

			render->idle = &idle;
			render->idle_param = &status;
			render->mainLoop(NULL);

			delete render;
			delete ag;


			// controller = NULL;
			if (!do_all) return 0;
		}
	}
	return 0;
}
