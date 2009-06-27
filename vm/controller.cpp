#include "common.h"
#include "controller.h"

hohmann::hohmann(trace_generator *trace, uint32_t instance):Icontroller(trace, instance) {
  _score_addr = 0x0;
  _fuel_addr = 0x1;
  _vx_addr = 0x2;
  _vy_addr = 0x3;
  _target_orbit_addr = 0x4;
  _instance_addr = 0x3E80;
  _delta_vx_addr = 0x2;
  _delta_vy_addr = 0x3;
}


void hohmann::calculate_action(double *dvx, double *dvy) {
  
}

bool hohmann::step(uint32_t time_step) {
  if (time_step == 0) // init 
  {
	vm->input_ports[_instance_addr] = _instance;
	_trace->add_command(time_step, _instance_addr, _instance, vm->output_ports[_score_addr]);
	return false;
  }
  
  if (vm->output_ports[_score_addr]) {
	return _trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
  }
  
  double dvx, dvy;
  calculate_action(&dvx, &dvy);
  
  if (dvx != vm->input_ports[_delta_vx_addr] ) //need a change
  {
	vm->input_ports[_delta_vx_addr] = dvx;
	_trace->add_command(time_step, _delta_vx_addr, dvx, vm->output_ports[_score_addr]);
  }
  
  if (dvy != vm->input_ports[_delta_vy_addr] ) //need a change
  {
	vm->input_ports[_delta_vy_addr] = dvy;
	_trace->add_command(time_step, _delta_vy_addr, dvy, vm->output_ports[_score_addr]);
  }
  
  return false;
}

void hohmann::monitor() {
  	cout << "fuel : "<< vm->output_ports[_fuel_addr] << endl;
	cout << "x : "<< vm->output_ports[_vx_addr] << endl;
	cout << "y : "<< vm->output_ports[_vy_addr] << endl;
	cout << "radius : " << sqrt((vm->output_ports[_vx_addr] * vm->output_ports[_vx_addr]) + (vm->output_ports[_vy_addr] * vm->output_ports[_vy_addr])) << endl;
}