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

void idle(void* user_param) {
	struct main_status* status = (struct main_status*)user_param;
	vm_state* vm = status->ag->vm;
	agent* ag = status->ag;

	renderer* render = status->render;
	double fuel;
	static int time_step = 0;
	if (time_step == 2)
		ag->apply_proportionnal_speed(0.5, 0);
	else if (time_step == 3)
		vm->set_speed(0, 0);
	else if (time_step == 1000)
		ag->apply_proportionnal_speed(-0.25, 0);
	else if (time_step == 1001)
		vm->set_speed(0, 0);

	vm->step();
	render->main_sat.pos_x = status->vm->get_pos_x();
	render->main_sat.pos_y = status->vm->get_pos_y();
	fuel = status->vm->get_fuel();
	render->set_fuel(fuel);
	render->set_sat(vm->get_targets());
	time_step++;
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

			render = new renderer();
			agent* ag = new agent(instance);

			vm = ag->vm;
			vm->step();
			render->add_radius(vm->get_radius());
			render->set_max_fuel(vm->get_fuel());
			vm->reset();

			status.vm = vm;
			status.render = render;
			status.ag = ag;
			render->idle = &idle;
			render->idle_param = &status;
			render->mainLoop(NULL);

			delete vm;
			delete render;

			// controller = NULL;
			if (!do_all) return 0;
		}
	}
	return 0;
}
