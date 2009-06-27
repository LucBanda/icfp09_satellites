#include "common.h"
#include "vm_state.h"
#include "trace_generator.h"
#include "controller.h"

int main (int argc, char** argv)
{
  if (argc < 2) {
	cerr << "please enter first the input file name and then the instance id" << endl;
	exit(-1);
  }
	
  uint32_t instance = atoi(argv[2]);
  
  vm_state _vm;
  vm = &_vm;
  vm->load_file(argv[1]);
  
  ostringstream output_name;
  output_name << dec << instance <<".osf"<< endl;
  
  trace_generator trace(instance, (char*)output_name.str().c_str());
  hohmann controller(&trace, (double)instance);
  
  uint32_t time_step = 0;
  bool stop;
  do {
    
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
	  vm->step();
	}
	
	
	stop = controller.step(time_step);
	usleep(1000);
	cout << "\x1b[2J\x1b[H"<< flush;
	controller.monitor();
	time_step ++ ;
	
	
  } while (!stop);
  
}
