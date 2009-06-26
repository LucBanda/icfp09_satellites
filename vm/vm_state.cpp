#include "common.h"
#include "vm_state.h"
#include "instructions.h"

vm_state *vm_state::_instance = NULL;
	
vm_state::vm_state(){}


void vm_state::load_file(char* file){
  int frame_number;
  struct stat results;
  fstream binary_file(file,ios::binary|ios::in);
  if (!binary_file.is_open()) {
	cerr << "unable to open the file : "<< file << endl;
	return;
  }
  stat(file, &results);
  frame_number = results.st_size / 12;
  cerr << "frame number" << frame_number << endl;
  for (int i = 0; i< frame_number; i++) {
	if (i %2 == 0) {
	  uint32_t raw;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  cout << hex << memory[i]<<endl;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  cout << hex<<raw <<endl;
	  code[ADDRESS_RANGE] = instruction::parse(raw);
	} else {
	  uint32_t raw;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  cout << hex<<raw <<endl;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  cout << hex<<memory[i] <<endl;
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
  
}