#include "common.h"
#include "bin.h"
#include "renderer.h"
#include "sys/time.h"
#include "openga.hpp"

struct main_status {
	vm_state *vm;
	renderer *render;
};

void idle(void *user_param)
{
	struct main_status *status = (struct main_status *)user_param;
	vm_state *vm = status->vm;
	renderer *render = status->render;
	double fuel;

	vm->step();
	render->main_sat.pos_x = status->vm->get_pos_x();
	render->main_sat.pos_y = status->vm->get_pos_y();
	fuel = status->vm->get_fuel();
	render->set_fuel(fuel);
	render->set_sat(vm->get_targets());
}


int main (int argc, char** argv)
{
	bool do_all;
	vm_state *vm=NULL;
	struct main_status status;
	renderer *render;

	if (argc < 2) {
		cerr << "please enter the scenario id" << endl;
		exit(-1);
	}
	uint32_t instance=1001;

	if (strcmp(argv[1], "all") != 0) {
	  instance = atoi(argv[1]);
	  do_all =false;
	} else do_all = true;
	
	
	for (int i = 1; i < 5;i++) {
	  for (int j = 1; j<5;j++) {
		
		if (do_all) {
			instance = i*1000 + j;
		}

		render = new renderer();
		vm = bin_factory(instance);
		vm->step();
		render->add_radius(vm->get_radius());
		vm->reset();

		status.vm = vm;
		status.render = render;
		render->idle = &idle;
		render->idle_param = &status;
		render->mainLoop(NULL);

		delete vm;
		delete render;

		//controller = NULL;
		if (!do_all)
		  return 0;
	  }
	}
	return 0;
}
