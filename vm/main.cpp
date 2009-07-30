#include "common.h"
#include "vm_state.h"
#include "trace_generator.h"
#include "controller.h"
#include "meet_and_greed.h"
#include "excentric.h"
#include "renderer.h"
#include "sys/time.h"
#include "clear_sky.h"
#include "bin.h"

int main (int argc, char** argv) 
{
	double total_score = 0;
	Icontroller *controller = NULL;
	char input_file_name[50], filename[50];
	bool do_all;
	if (argc < 3) {
		cerr << "please enter first the problem binary path and then the scenario id" << endl;
		exit(-1);
	}
	uint32_t instance=1001;
	if (strcmp(argv[2], "all") != 0) {
	  instance = atoi(argv[2]);
	  do_all =false;
	} else do_all = true;
	
	trace_generator *trace;
	
	for (int i = 1; i < 5;i++) {
	  for (int j = 1; j<5;j++) {
		if (do_all) {
		  instance = i*1000 + j;
		  }
#ifdef GENERATE
		vm = new vm_state(instance);
		sprintf(input_file_name, "%s/bin%d.obf", argv[1], instance/1000);
		vm->load_file(input_file_name);
#else		
	sprintf(input_file_name, "%s/bin%d.obf", argv[1], instance/1000);
#endif
		sprintf(filename, "%d.osf", instance);
			
		trace = new trace_generator(instance, filename);
		
		if ((instance < 1005) && (instance > 1000)) {
#ifndef GENERATE
			vm = new bin_1;
			vm->load_file(input_file_name);
#endif
			controller = new hohmann(trace, (double)instance);
}			
		if ((instance < 2005) && (instance > 2000)) {
#ifndef GENERATE
			
			vm = new bin_2;
			vm->load_file(input_file_name);
#endif

			controller = new meetandgreed(trace, (double)instance);
		}
		if ((instance < 3005) && (instance > 3000)) {
#ifndef GENERATE
			
			vm = new bin_3;
			vm->load_file(input_file_name);
#endif
		  
			controller = new excentric(trace, (double)instance);
		}
		if ((instance < 4005) && (instance > 4000)) {
#ifndef GENERATE
			
			vm = new bin_4;
			vm->load_file(input_file_name);
#endif
			controller = new clear_sky(trace, (double)instance);
			
		}
		
		renderer::getInstance();
		uint32_t time_step = 0;
		bool stop = false;
		int count_fps = 0;
		int count = 0;
		struct timeval time, saved_time = {0};

		do {
			renderer::getInstance()->lock();
			time_step++;
			
			vm->step();
#ifdef GENERATE
			if (time_step > 2) {
				delete vm;
				return 0;
			} else {renderer::getInstance()->unlock(); continue;}
#endif			
			stop = controller->step(time_step);

			//cerr << "\x1b[2J\x1b[H";
			//controller->monitor();
			renderer::getInstance()->unlock();
			
			int timer = renderer::getInstance()->get_timer();
			if (timer != 0)
			usleep(renderer::getInstance()->get_timer() * 10 - 10);
			
			if (renderer::getInstance()->get_fps_toggle()) {
				count_fps ++;
				gettimeofday(&time, NULL);
				if (time.tv_sec != saved_time.tv_sec ) {
					cout << count_fps << " FPS ( " << count << " )\n";
					count_fps = 0;
					saved_time = time;
				}
			
				count ++;
			} else count = 0;
		} while (!stop);
		total_score += controller->get_score();
		cout << "score : " << controller->get_score() << " (" << total_score << " ) total step : " << count << "\n";
		renderer::kill();
		usleep(10);
		delete controller;
		delete trace;
		delete vm;
		vm = NULL;
		trace=NULL;
		controller = NULL;
		if (!do_all)
		  exit(0);
	  }
	}
}
