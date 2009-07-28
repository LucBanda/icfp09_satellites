#include "common.h"
#include "vm_state.h"
#include "instructions.h"


vm_state *vm=NULL;
	
vm_state::vm_state(int instance){
  pc = 0;
  status = 0;
  memory = (double*)malloc(sizeof(double) * ADDRESS_RANGE);
  output_ports = (double*)malloc(sizeof(double) * ADDRESS_RANGE);
  input_ports = (double *)malloc(sizeof(double) * ADDRESS_RANGE);
  state = (int *)malloc(sizeof(int) * ADDRESS_RANGE);
  code = (instruction **) malloc(sizeof(instruction*) * ADDRESS_RANGE);
  memset(output_ports, 0, sizeof(double) * ADDRESS_RANGE);
  memset(input_ports, 0, sizeof(double) * ADDRESS_RANGE);
  memset(memory, 0, sizeof(double) * ADDRESS_RANGE);
  memset(code, 0, sizeof(instruction *) * ADDRESS_RANGE);
  memset(state,0, sizeof(int) * ADDRESS_RANGE);
  pass = 0;
  min_global = 1<<14;
  max_global = 0;
  min_out_port = 1<<14;
  max_out_port = 0;
  _instance = instance;
  min_global = 65535;
  max_global = 0;

#ifdef GENERATE
  
#endif
}
vm_state::vm_state(){
 code = NULL;
 state = NULL;
}
vm_state::~vm_state() {
#ifdef GENERATE
  cout << "status = lstatus;\n}\n#endif" << endl;
#endif
	free(memory);
	free(output_ports);
	free(input_ports);
	if (state)
		free(state);
	if (code)
		free(code);
}

vm_state *vm_state::clone() {
  vm_state *cloned = new vm_state;
  
  memcpy(cloned->memory,	memory, 	sizeof(double) * ADDRESS_RANGE);
  memcpy(cloned->output_ports,	output_ports, 	sizeof(double) * ADDRESS_RANGE);
  memcpy(cloned->input_ports, 	input_ports, 	sizeof(double) * ADDRESS_RANGE);  
  memcpy(cloned->code, 		code,		sizeof(instruction*) * ADDRESS_RANGE);
  cloned->status = status;
  cloned->pc = pc;
  cloned->min_global = min_global;
  cloned->max_global = max_global;
  cloned->min_out_port = min_out_port;
  cloned->max_out_port = max_out_port;
  return cloned;
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
#ifndef GENERATE
	  binary_file.clear();
	  double value;
	  binary_file.read(reinterpret_cast<char *> (&value),8);
	  if (i >= min_global && i < max_global)
		  memory[i-min_global] = value;
#else
  	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
#endif
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
#ifdef GENERATE
	  code[i] = instruction::parse(raw);
#endif
	} else {
	  uint32_t raw;
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&raw),4);
#ifndef GENERATE
	  binary_file.clear();
	  double value;
	  binary_file.read(reinterpret_cast<char *> (&value),8);
	  if (i >= min_global && i < max_global)
		  memory[i-min_global] = value;
#else
	  binary_file.clear();
	  binary_file.read(reinterpret_cast<char *> (&memory[i]),8);
	  code[i] = instruction::parse(raw);
#endif
	  
	}
  }
  binary_file.clear();
  
  binary_file.close();
#ifdef GENERATE
  for (int i = frame_number; i<ADDRESS_RANGE; i++) {
	memory[i] = 0;
	code[i] = instruction::parse(0);
  }
#endif
  
}

void vm_state::step() 
{
	int j;
	for (int i=0; i<ADDRESS_RANGE;i++)
	{
		code[pc]->execute(this);
		pc++;
		if (pc == ADDRESS_RANGE)
			pc = 0;
	}

	if (pass == 0) {
		for (j=0; j< ADDRESS_RANGE; j++) {
			if ((min_global == 65535) && (state[j] & FIRST_READ) && (state[j] & WRITE)) {
				min_global = j;
				break;
			}
		}
		for (j=ADDRESS_RANGE; j>= 0; j--) {
			if ((max_global == 0) && (state[j] & FIRST_READ) && (state[j] & WRITE)) {
				max_global = j;
				break;
			}
		}
		cout << "#include \"bin.h\"\n#ifndef GENERATE \n"; 
		cout << "bin_"<<_instance/1000<<"::bin_"<<_instance/1000<< "(){\n";
		cout << "min_out_port = "<< min_out_port <<";\n";
		cout << "max_out_port = "<< max_out_port <<";\n";
		cout << "output_ports = (double*)malloc(sizeof(double) * (max_out_port - min_out_port+1));\n";
		cout << "min_global = "<< min_global <<";\n";
		cout << "max_global = "<< max_global <<";\n";
		cout << "memory = (double*)malloc(sizeof(double) * (max_global - min_global+1));\n";
		cout << "input_ports = (double*)malloc(sizeof(double) * 4);\n";
		
		cout <<"status = 0;\n memset(output_ports, 0, sizeof(double) * (max_out_port - min_out_port +1));\n \
		memset(input_ports, 0, sizeof(double) * 4);\n \
		memset(memory, 0, sizeof(double) * (max_global-min_global+1));\n";
		cout << "}" << endl;
		cout <<"void bin_"<<_instance/1000<<"::step() {\nbool lstatus = status;" << endl;
		
		cout << "double ";
		for ( j=0; j < ADDRESS_RANGE; j++) {
			if (state[j] & FIRST_WRITE) {
				cout << "local_"<<j<<",";
			}
		}
		cout << "local_dummy;"<<endl;
		
		cout << "const double ";
		for (j = 0; j< ADDRESS_RANGE; j++) {
			if ((state[j] & READ) && !(state[j] & WRITE))
				cout << "const_"<<j<<"="<<memory[j]<<",";
		}
		cout << "const_dummy=0;"<<endl;
	}

  pass++;
}