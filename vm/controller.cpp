#include "common.h"
#include "controller.h"
#include <complex>
#include "renderer.h"

hohmann::hohmann(trace_generator *trace, double instance) : Icontroller(trace) {
  
  _delta_vx_addr = 0x2;
  _delta_vy_addr = 0x3;
  _instance_addr = 0x3E80;
  _score_addr = 0;
  vm->input_ports[_instance_addr] = _instance = instance;
  _trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  me = new satellite(0x2, 0x3, NULL, new satellipse);
  renderer::getInstance()->add_sat(me);
  
}



complex<double> hohmann::calculate_action(uint32_t time_step) {
  
  me->update(time_step);
  //if (time_step <2) return complex<double>(0,0);
  return me->travel_to(vm->output_ports[0x4]);
  
}


bool hohmann::step(uint32_t time_step) {
  
  if (vm->output_ports[_score_addr]) {
	return _trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
  }
  
  if (time_step == 1) {
	renderer::getInstance()->set_max_fuel(vm->output_ports[_fuel_addr]);
  } 
  renderer::getInstance()->set_fuel(vm->output_ports[_fuel_addr]);
  
  if (time_step == 1) {
	  renderer::getInstance()->add_radius(vm->output_ports[0x4]);
  }
  double dvx, dvy;
  complex<double> action = calculate_action(time_step);
  dvx = real(action);
  dvy = imag(action);
  
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
  	/*cout << "fuel : "<< vm->output_ports[_fuel_addr] << endl;
	cout << "x : "<< vm->output_ports[_vx_addr] << endl;
	cout << "y : "<< vm->output_ports[_vy_addr] << endl;
	cout << "radius : " << sqrt((vm->output_ports[_vx_addr] * vm->output_ports[_vx_addr]) + (vm->output_ports[_vy_addr] * vm->output_ports[_vy_addr])) << endl;
	cout << "target : " << vm->output_ports[_target_orbit_addr] << endl;*/
	cerr << "to goal : " << sqrt((vm->output_ports[0x2] * vm->output_ports[0x2]) + (vm->output_ports[0x3] * vm->output_ports[0x3])) - vm->output_ports[0x4] << "\n";
}