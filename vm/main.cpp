#include "common.h"
#include "vm_state.h"
#include "trace_generator.h"
#include "controller.h"
#include "meet_and_greed.h"
#include "renderer.h"

int main (int argc, char** argv)
{
  Icontroller *controller = NULL;
  
  if (argc < 3) {
	cout << "please enter first the problem binary filename and then the scenario id" << endl;
	exit(-1);
  }
  
  uint32_t instance = atoi(argv[2]);
  
  vm_state _vm;
  vm = &_vm;
  vm->load_file(argv[1]);

  ostringstream output_name;
  output_name << dec << instance <<".osf";
  
  trace_generator trace(instance, (char*)output_name.str().c_str());
  if ((instance < 1005) && (instance > 1000))
	controller = new hohmann(&trace, (double)instance);
  if ((instance < 2005) && (instance > 2000))
	controller = new meetandgreed(&trace, (double)instance);

  
  //renderer::getInstance();
  uint32_t time_step = 0;
  bool stop;
  do {
    
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
	  vm->step();
	}
	
	
	stop = controller->step(time_step);
	controller->monitor();
	time_step ++ ;
	
	
  } while (!stop);
  
}
