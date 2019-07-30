#include "common.h"
#include "vm_state.h"

vm_state::vm_state()
{
}

void vm_state::reset()
{
	memset(output_ports, 0, sizeof(double) * (max_out_port - min_out_port+1));
	memset(input_ports, 0, sizeof(double) * 4);
	memset(memory, 0, sizeof(double) * (max_global - min_global+1));
	input_ports[instance_addr] = _instance;
	status = 0;
	pc = 0;
}

void vm_state::step()
{

}

int vm_state::get_instance()
{
	return input_ports[instance_addr];
}

double vm_state::get_fuel()
{
	return output_ports[fuel_addr];
}

double vm_state::get_score()
{
	return output_ports[score_addr];
}

double vm_state::get_pos_x()
{
	return output_ports[pos_x_addr];
}

double vm_state::get_pos_y()
{
	return output_ports[pos_y_addr];
}

double vm_state::get_radius()
{
	return 0;
}

vector<satellite> vm_state::get_targets()
{
	vector<satellite> empty;
	return empty;
}