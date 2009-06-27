#ifndef TRACE_GENERATOR_H
#define TRACE_GENERATOR_H

#define MAGIC 0xCAFEBABE
#define TEAM_ID 360
#define MAX_TIME_STEP 3000000
#define MAX_ACTUATOR  10


struct _port_value_mapping {
  uint32_t address;
  double value;
};

struct _frame {
  uint32_t time_step;
  uint32_t count;
  struct _port_value_mapping port_map[MAX_ACTUATOR];
};

struct _file_struct {
  uint32_t magic;
  uint32_t team_id;
  uint32_t scenario_id;
  struct _frame frame[MAX_TIME_STEP];
};

class trace_generator {
  private :
	uint32_t current_frame_id;
	uint32_t current_port_map_id;
	struct _file_struct file_struct;
	char * _output_file;
	
	void next_frame(uint32_t time_step);
  public:
	trace_generator(uint32_t scenario_id, char* output_file);
	bool add_command(uint32_t time_step, uint32_t address, double value, double score);
	void print_file();
};

#endif