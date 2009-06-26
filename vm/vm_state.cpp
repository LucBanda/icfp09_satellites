#include "common.h"
#include "vm_state.h"
#include "instructions.h"

vm_state *vm_state::_instance = NULL;
	
vm_state::vm_state(){}


void vm_state::load_file(string file){
  int frame_number;
  fstream binary_file(file.c_str(),ios::binary|ios::in);
  
  frame_number = binary_file.tellg() / 12;
  for (int i = 0; i< frame_number; i++) {
	if (i %2 == 0) {
	  uint32_t raw;
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  cout << memory[i]<<endl;
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  cout << raw <<endl;
	  code[ADDRESS_RANGE] = instruction::parse(raw);
	} else {
	  uint32_t raw;
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
	  cout << raw <<endl;
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  cout << memory[i] <<endl;
	  code[i] = instruction::parse(raw);
	}
  }
  binary_file.close();
  for (int i = frame_number; i<ADDRESS_RANGE; i++) {
	memory[i] = 0;
	code[i] = instruction::parse(0);
  }
}

void vm_state::step() {
  
}