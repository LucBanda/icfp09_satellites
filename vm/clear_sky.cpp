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
	me = new satellite(0x2, 0x3, NULL, new satellipse());
	fuelling = new satellite(0x4,0x5, me, new satellipse());
	for (int i = 0; i<11; i++) {
	  target[i] = new satellite(3*i+0x7, 3*i+0x8, me, new satellipse());
	  if (i!=10)
	  renderer::getInstance()->add_sat(target[i]);
	}
	renderer::getInstance()->add_sat(me);
	renderer::getInstance()->add_sat(fuelling);
	
	vm->input_ports[_instance_addr] = _instance = instance;
	_trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
	
	first_not_checked = -1;
  
}


complex<double> clear_sky::calculate_action(uint32_t time_step) {


  int i;
  complex<double> command = complex<double> (0,0);
  
  //update all
  me->update(time_step);
  for (i=0; i<11; i++) {
	target[i]->update(time_step);
  }
  fuelling->update(time_step);
  
  
  //cout << "state : " << me->state() << "first not checked " << first_not_checked << "checked ? " << vm->output_ports[3*first_not_checked+0x9] << endl;
  
  if ((first_not_checked != -1) && ((vm->output_ports[3*first_not_checked+0x9]) == 1.0)) {
	command = me->set_circular_orbit();
	  if (abs(command) < 1E-20) return command;
	  for (i =0;i< 11;i++) {
	  //  cout << vm->output_ports[3*i+0x9] << " ";
	  if (vm->output_ports[3*i+0x9] < 1E-20) {
		first_not_checked = i;
		break;
	  }
	}
  } else if (first_not_checked == -1) {
	first_not_checked = 0;
  } else {
	//cout << "meet " << first_not_checked << endl;
	command = me->meet(target[first_not_checked], false);
  }
  //cout << "to target " << first_not_checked << endl;
  

	
  
  return command;
  
}
  
bool clear_sky::step(uint32_t time_step) {
  
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

void clear_sky::monitor() {
  /*cerr << "current_radius : " << abs(me->position()) << "\n";
  cerr << "relative distance to target : " << abs(me->position() - target[first_not_checked]->position()) << "\n";
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
  cerr << "\n";*/
}