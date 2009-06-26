#include "common.h"
#include "vm_state.h"

int main (int argc, char** argv)
{
  assert (argv[1]);
  vm->load_file(argv[1]);
  
  while (1) {
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
	  vm->step();
	}
	getchar();
  }
}