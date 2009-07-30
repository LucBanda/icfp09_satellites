#include "common.h"
#include "satellite.h"
#include "renderer.h"

double to_range(double in_val) {
  double val = in_val;
  while(val > M_PI) val -=2*M_PI;
  while(val < -M_PI) val +=2*M_PI;
  return val;
}

void satellite::update(uint32_t time_step)
{
  if ((time_step > 1) && (_state == INIT)) _state = ORBITING;
  
  _relative_position = complex<double>(vm->output_ports[addr_x], vm->output_ports[addr_y]);
  
  if (_main == NULL) {
	_position = -_relative_position;
	_trajectoire->add_position(_position, time_step);
  } else
	_position = _main->position() + _relative_position;
  _time_step = time_step;
  _speed = _position - _old_position;
  _orbit = abs(_position);
  
  _old_position = _position;
  
  if (_time_step > _trajectoire->known_time_steps()) {
	_trajectoire->add_position(_position, time_step);
  }
  if (_trajectoire->known_time_steps() > 1) {
	complex<double> end_speed = _trajectoire->get_pos_at(_trajectoire->known_time_steps()) - _trajectoire->get_pos_at(_trajectoire->known_time_steps()-1);
	if (abs(end_speed) > abs(_max_speed))
	  _max_speed = end_speed;
  }
}

complex<double> satellite::travel_to(double target_orbit, complex<double> *target_position, bool simulate)
{
  complex<double> delta_v(0.0,0.0);
  if ((_state == ORBITING) || (simulate)) {

	double dv_abs = sqrt(MU/abs(_position))*(sqrt(2.0*target_orbit/(target_orbit+abs(_position)))-1.0);
	
	if (abs(to_range(arg(_position) - M_PI/2 - arg(_speed))) < M_PI/2)
		delta_v =  polar(dv_abs, arg(_position) - M_PI/2);
	else {
		delta_v =  polar(dv_abs, arg(_position) + M_PI/2);
	}
	_ignition_time = _time_step;

	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1.0-sqrt(2*abs(_position)/(target_orbit+abs(_position))));
	
	if (abs(to_range(arg(_position) - M_PI/2 - arg(_speed))) < M_PI/2)
		_speed_back = polar(dv_abs, arg(_position) + M_PI/2);
	else
		_speed_back = polar(dv_abs, arg(_position) - M_PI/2);
	_stop_time = _ignition_time +  time_to_travel_to(target_orbit) ;
	if (target_position)
	  *target_position = polar(abs(target_orbit), arg(_position) + M_PI);
	
	if (!simulate)
	  _state = TRAVELLING;
  } else if (_state == TRAVELLING) {
	if (_stop_time == _time_step+1) {
	 _state = ORBITING;
	 delta_v = _speed_back;
	}
  }
  return delta_v;
}

complex<double> satellite::meet(satellite *target, bool track_target)
{

  if (_state == ORBITING) {
	
	
	complex<double>  position_to_arrive, needed_delta_v;
	complex<double> target_pos =  target->position();
	double target_orbit =abs(target_pos);
	if (target_orbit == 0)
		return complex<double> (0,0);
	uint32_t t = 0;
	bool forward = true;
	int factor = time_to_travel_to(target_orbit);
	do {
		
		target_pos = target->position_at(t);
		target_orbit = abs(target_pos);
		
		if (t > time_to_travel_to(target_orbit)) {
			if (forward)
				factor /= 2;
			t-=factor;
			forward = false;
		} else {
			if (!forward)
				factor /=2;
			t+=factor;
			forward = true;
		}
	if (t < 0) return complex<double> (0,0);
	} while (factor);
	
	needed_delta_v = travel_to(target_orbit, &position_to_arrive, true);

	_old_target_pos = target_pos;
	renderer::getInstance()->add_position(target_pos);
	

	if ((abs(position_to_arrive - target_pos) < 4000.0)) {
	  _state = TRAVELLING; //validate simulation

	  return needed_delta_v;
	}
  } else if ((_state == TRAVELLING)) {
      complex<double> command = travel_to(0, NULL, false);
      if (abs(command) > 1E-15) {
	  _state = DOCKING;
      }
      return complex<double>(0,0);
  } else if ((_state == DOCKING) || (_state == ADJUSTING)){
	  if (_state == ADJUSTING) {
		_state = DOCKING;
		return complex<double>(0.0,0.0);
	  }
	  complex< double> rel_speed = _speed - target->speed();
      if (abs(target->position() - _position)<500) {
		if (abs(rel_speed) > 1.0) {
		  _state = ELLIPTIC;
		}
		if (track_target) {
		  return -rel_speed;
		}
      } else  {
		if ((abs(rel_speed) < 19.5) || (abs(rel_speed) > 20.5) || (abs(arg(rel_speed) - arg(_speed)) < M_PI/2))  {
		  rel_speed -= polar(20.0, arg(target->relative_position()));
		  _state = ADJUSTING;
		  
		  return -rel_speed;
		}
      }
  }
  
  return complex<double>(0.0,0.0);
}

complex<double> satellite::set_circular_orbit() {
  
	
	_old_target_pos = 0;
	complex<double> pos2, pos1 = _trajectoire->get_pos_at(_time_step - 2);
	pos2 = _trajectoire->get_pos_at(_time_step - 1);
	complex<double> lspeed = _position - pos2;
	if (abs(lspeed) < abs( pos2 - pos1)) {
	    _state = ORBITING;
	    return polar(sqrt(MU / abs(_position)), arg(_position) - M_PI/2) - (_position-pos2);
	}
	return complex<double>(0,0);
}

uint32_t satellite::time_to_travel_to(double target_orbit)
{
    return M_PI * sqrt(pow(abs(_position) + target_orbit,3)/(8*MU)) - 1;
  
}
complex<double> satellite::step_forward(uint32_t step) {
  vm_state *clone;
  clone = vm->clone();
  for (uint32_t i = 0; i< step; i++) {
	clone->step();
  }
  complex<double> pos;
  if (_main == NULL)
	pos = -complex<double>(clone->output_ports[addr_x], clone->output_ports[addr_y]);
  else
	pos = -complex<double>(clone->output_ports[_main->addr_x], clone->output_ports[_main->addr_y]) + complex<double>(clone->output_ports[addr_x], clone->output_ports[addr_y]);
  
  delete clone;  
  return pos;
  
}

complex<double> satellite::position_at(uint32_t time_step_forward)
{
  if (time_step_forward + _time_step > _trajectoire->known_time_steps()) {
	uint32_t known_time = _trajectoire->known_time_steps();
	for (unsigned int i=known_time; i<= time_step_forward + _time_step; i++) {
	  if (vm_clone == NULL)
		vm_clone = vm->clone();
	  vm_clone->step();
	  
	  if (_main == NULL)
		_trajectoire->add_position(-complex<double>(vm_clone->output_ports[addr_x], vm_clone->output_ports[addr_y]) , i);
	  else
		_trajectoire->add_position(-complex<double>(vm_clone->output_ports[_main->addr_x], vm_clone->output_ports[_main->addr_y]) + complex<double>(vm_clone->output_ports[addr_x], vm_clone->output_ports[addr_y]) , i);
	}
	
  } 
  return _trajectoire->get_pos_at(_time_step + time_step_forward);
}
