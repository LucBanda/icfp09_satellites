#include "common.h"
#include "meet_and_greed.h"
#include "renderer.h"
#include "ellipse.h"

meetandgreed::meetandgreed(trace_generator *trace, double instance):Icontroller(trace) {
  if (instance < 2005) {
	_score_addr = 0x0;
	_fuel_addr = 0x1;
	
	_delta_vx_addr = 0x2;
	_delta_vy_addr = 0x3;
	_instance_addr = 0x3E80;
	me = new satellite(0x2, 0x3);
	target = new satellite(0x4, 0x5, me);
	renderer::getInstance()->add_sat(me);
	renderer::getInstance()->add_sat(target);
	
	vm->input_ports[_instance_addr] = _instance = instance;
	_trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  } else {
	_score_addr = 0x0;
	_fuel_addr = 0x1;
	
	_delta_vx_addr = 0x2;
	_delta_vy_addr = 0x3;
	_instance_addr = 0x3E80;
	me = new satellite(0x2, 0x3);
	target = new satellite(0x4, 0x5, me, new satellipse());
	renderer::getInstance()->add_sat(me);
	renderer::getInstance()->add_sat(target);
	
	vm->input_ports[_instance_addr] = _instance = instance;
	_trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  }
  
}


complex<double> meetandgreed::calculate_action(uint32_t time_step) {
  cout << "\x1b[2J\x1b[H";
  me->update(time_step);
  target->update(time_step);
  return me->meet(target);
  
}
  
bool meetandgreed::step(uint32_t time_step) {
  
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

void meetandgreed::monitor() {
  cout << "relative distance to target " << abs(me->position() - target->position()) << endl;
  cout << "score : " << vm->output_ports[_score_addr] << endl;
}