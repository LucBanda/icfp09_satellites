#include "common.h"
#include "excentric.h"
#include "renderer.h"

excentric::excentric(trace_generator *trace, double instance) :Icontroller(trace)
{
	_score_addr = 0x0;
	_fuel_addr = 0x1;
	
	_delta_vx_addr = 0x2;
	_delta_vy_addr = 0x3;
	_instance_addr = 0x3E80;
	vm->input_ports[_instance_addr] = _instance = instance;
	_trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
	
	me = new satellite(0x2, 0x3, NULL, new satellipse());
	target = new satellite(0x4, 0x5, me, new satellipse());
	renderer::getInstance()->add_sat(me);
	renderer::getInstance()->add_sat(target);
	

}

complex<double> excentric::calculate_action(uint32_t time_step) {

  me->update(time_step);
  target->update(time_step);
  return me->meet(target);
  
}
  

bool excentric::step(uint32_t time_step) {
  
  if (vm->output_ports[_score_addr]) {
	_trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
    return true;
  }
    
  if (time_step == 1) {
	renderer::getInstance()->set_max_fuel(vm->output_ports[_fuel_addr]);
  } 
  renderer::getInstance()->set_fuel(vm->output_ports[_fuel_addr]);
  
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

void excentric::monitor() {
  cerr << "current_radius : " << abs(me->position()) << "\n";
  cerr << "relative distance to target : " << abs(me->position() - target->position()) << "\n";
  cerr << "score : " << vm->output_ports[_score_addr] << "\n";
  cerr << "fuel : " << vm->output_ports[_fuel_addr] << "\n";
  cerr << "state : ";
  switch (me->state()) {
	case INIT : cerr << "INIT"; break;
	case ORBITING : cerr << "ORBITING"; break;
	case TRAVELLING : cerr << "TRAVELLING"; break;
	case DOCKING : cerr << "DOCKING"; break;
	case ADJUSTING : cerr << "ADJUSTING"; break;
	case ELLIPTIC : cerr << "ELLIPTIC"; break;
	case STABILIZE : cerr << "STABILIZE"; break;
	default: cerr << "unknown"; break;
  }
}
