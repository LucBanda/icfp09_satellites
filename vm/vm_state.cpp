#include "common.h"
#include "vm_state.h"
#include "instructions.h"


vm_state *vm=NULL;
	
vm_state::vm_state(){
  pc = 0;
  status = 0;
}


void vm_state::load_file(char* file){
  int frame_number;
  struct stat results;
  fstream binary_file(file,ios::binary|ios::in);
  if (!binary_file.is_open()) {
	cerr << "unable to open the file : "<< file << endl;
	exit(-1);
  }
  stat(file, &results);
  frame_number = results.st_size / 12;
  //cerr << "frame number" << frame_number << endl;
  for (int i = 0; i< frame_number; i++) {
	if (i %2 == 0) {
	  uint32_t raw;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  code[i] = instruction::parse(raw);
	} else {
	  uint32_t raw;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  code[i] = instruction::parse(raw);
	}
  }
  binary_file.close();
  for (int i = frame_number; i<ADDRESS_RANGE; i++) {
	memory[i] = 0;
	code[i] = instruction::parse(0);
  }
}

void vm_state::step() 
{
	  code[pc]->execute();
	  pc++;
	  if (pc == ADDRESS_RANGE)
		pc = 0;
}