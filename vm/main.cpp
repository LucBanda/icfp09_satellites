#include "common.h"
#include "vm_state.h"

int main (int argc, char** argv)
{
  if (argc < 2) {
	cerr << "please enter first the input file name and then the instance id" << endl;
	exit(-1);
  }
	
  int instance = atoi(argv[2]);
  
  vm->load_file(argv[1]);
  char output_name[256];
  
  snprintf(&output_name, "%d.osf", instance);
  trace_generator trace(output_name, instance);
  Icontroller controller(&trace);
  uint32_t time_step = 0;

  while (controller.step(time_step++)) {
    
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
	  vm->step();
	}
	
	controller.monitor();
	
	cout << "\x1b[2J\x1b[H"<< flush;
	/*cout << "fuel : "<< vm->output_ports[0x1] << endl;
	cout << "x : "<< vm->output_ports[0x2] << endl;
	cout << "y : "<< vm->output_ports[0x3] << endl;
	cout << "radius : " << sqrt((vm->output_ports[0x2] * vm->output_ports[0x2]) + (vm->output_ports[0x3] * vm->output_ports[0x3])) << endl;*/
	
	cerr << "-------------------------------------" << endl;
  }
  
}
