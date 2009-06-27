#include "common.h"
#include "trace_generator.h"


trace_generator::trace_generator(uint32_t scenario_id, char* output_file) 
{
  file_struct.magic = MAGIC;
  file_struct.team_id = TEAM_ID;
  file_struct.scenario_id = scenario_id;
  _output_file = output_file;
  
  current_frame_id = 0;
  current_port_map_id = 0;
}

void trace_generator::next_frame(uint32_t time_step)
{
  file_struct.frame[current_frame_id].count = current_port_map_id;
  current_frame_id++;
  current_port_map_id=0;
  assert(current_port_map_id < MAX_ACTUATOR);
  if (current_frame_id == MAX_TIME_STEP) {
	cerr << "error time_step exceeding max value : " << time_step << endl;
	exit(-1);
  }
  file_struct.frame[current_frame_id].time_step = time_step;
  
}

bool trace_generator::add_command(uint32_t time_step, uint32_t address, double value, double score)
{
  if (time_step != file_struct.frame[current_frame_id].time_step)
	next_frame(time_step);
  
  if (score) {
	file_struct.frame[current_frame_id].count = 0;
	print_file();
	return true;
  }
  
  file_struct.frame[current_frame_id].port_map[current_port_map_id].address = address;
  file_struct.frame[current_frame_id].port_map[current_port_map_id].value = value;
  
  
  current_port_map_id++;
  return false;
}

void trace_generator::print_file() {
  fstream binary_file(file,ios::binary|ios::out);
  if (!binary_file.is_open()) {
	cerr << "unable to open the file : "<< file << endl;
	exit(-1);
  }
  //printing header
  binary_file.write(&struct_file.magic, 4);
  binary_file.write(&struct_file.team_id, 4);
  binary_file.write(&struct_file.scenario_id, 4);
  uint32_t cur_f_p=0;
  uint32_t cur_pm_p=0;
  
  while (struct_file.frame[cur_f_p].count !=0) {
	binary_file.write(&struct_file.frame[cur_f_p].time_step, 4);
	binary_file.write(&struct_file.frame[cur_f_p].count, 4);
	for (cur_pm_p = 0; cur_pm_p < struct_file.frame[cur_f_p].count; cur_pm_p++) {
	  binary_file.write(&struct_file.frame[cur_f_p].port_map[cur_pm_p].address, 4);
	  binary_file.write(&struct_file.frame[cur_f_p].port_map[cur_pm_p].value, 8);
	}
  }
  
  binary_file.close();
}