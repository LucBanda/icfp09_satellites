#include "common.h"
#include "vm_state.h"

int main (int argc, char** argv)
{
  assert (argv[1]);
  vm_state::instance()->load_file(string(argv[1]));
}