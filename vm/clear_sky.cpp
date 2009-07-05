#include "common.h"
#include "clear_sky.h"
#include "renderer.h"
#include "ellipse.h"

clear_sky::clear_sky(trace_generator *trace, double instance):Icontroller(trace) {
  
	_score_addr = 0x0;
	_fuel_addr = 0x1;
	_delta_vx_addr = 0x2;
	_delta_vy_addr = 0x3;
	_instance_addr = 0x3E80;
	me = new satellite(0x2, 0x3);
	fuelling = new satellite(0x4,0x5, me, new satellipse());
	for (int i = 0; i<11; i++) {
	  target[i] = new satellite(3*i+0x7, 3*i+0x8, me, new satellipse());
	  renderer::getInstance()->add_sat(target[i]);
	}
	renderer::getInstance()->add_sat(me);
	renderer::getInstance()->add_sat(fuelling);
	
	vm->input_ports[_instance_addr] = _instance = instance;
	_trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  
}


complex<double> clear_sky::calculate_action(uint32_t time_step) {

  me->update(time_step);
  for (int i=0; i<10; i++) {
	target[i]->update(time_step);
  }
  fuelling->update(time_step);
  return complex<double>(0,0);
  
}
  
bool clear_sky::step(uint32_t time_step) {
  
  if (vm->output_ports[_score_addr]) {
	_trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
    return true;
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

void clear_sky::monitor() {
  
}