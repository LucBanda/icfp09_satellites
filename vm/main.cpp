#include "common.h"
#include "vm_state.h"

int main (int argc, char** argv)
{
  assert (argv[1]);
  vm->load_file(argv[1]);
  
  vm->input_ports[0x3E80] = 1001;
  int vitesse = 0;
  while (1) {
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
	  vm->step();
	}
	cout << "\x1b[2J\x1b[H"<< flush;
	cout << "fuel : "<< vm->output_ports[0x1] << endl;
	cout << "x : "<< vm->output_ports[0x2] << endl;
	cout << "y : "<< vm->output_ports[0x3] << endl;
	
	
	cerr << "-------------------------------------" << endl;
	vm->input_ports[0x2] = vitesse++;
	vm->input_ports[0x3] = vitesse++;
	getchar();
  }
  
}
