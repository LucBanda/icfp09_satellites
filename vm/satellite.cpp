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
	
	delta_v =  polar(dv_abs, arg(_speed));
	_ignition_time = _time_step;

	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1.0-sqrt(2*abs(_position)/(target_orbit+abs(_position))));
	
	_speed_back = polar(dv_abs, arg(_speed) + M_PI);
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

complex<double> satellite::meet(satellite *target)
{

  if (_state == ORBITING) {
	
	double target_orbit =0;
	uint32_t arrival_time;
	complex<double>  position_to_arrive, needed_delta_v;
	complex<double> target_pos =  target->position();
	
	/*if (abs(target->position_at(arrival_time) - target->position_at(arrival_time -1)) < (abs(target->_max_speed) * 0.01)) //move on
	  return complex<double>(0.0,0.0);*/
	int i = 0;
	do {
	  //diff_in_target_orbit = abs(abs(target_pos) - target_orbit);

	  target_orbit = abs(target_pos);
	  if (target_orbit == 0)
		return complex<double> (0,0);
	  arrival_time = time_to_travel_to(target_orbit) + 2;
	  cerr << "target_orbit " << target_orbit << endl;
	  cerr << "arrival_time" << arrival_time << endl;
	  needed_delta_v = travel_to(target_orbit, &position_to_arrive, true);
	  target_pos = target->position_at(arrival_time);
	  cerr << "target_pos " << target_pos << endl;
	  cerr << "difference in target orbit :" << abs(abs(target_pos) - target_orbit) << endl;
	  i++;  

	} while ((abs(abs(target_pos) - target_orbit) > 7500.0) && (i<1000));
	
	_old_target_pos = target_pos;
	renderer::getInstance()->add_position(target_pos);
	renderer::getInstance()->unlock();
	renderer::getInstance()->lock();
	
	/*if (arrival_time > 100000)
	  return complex<double>(0,0);*/
	
	if ((abs(position_to_arrive - target_pos) < 15000.0)) {
	  _state = DOCKING; //validate simulation
	  return needed_delta_v;
	}
  } else if ((_state == DOCKING) || (_state == ADJUSTING)){
	  if (_state == ADJUSTING) {
		_state = DOCKING;
		return complex<double>(0.0,0.0);
	  }
	  complex< double> rel_speed = _speed - target->speed();
      if (abs(target->position() - _position)<1000) {
		if (abs(rel_speed) > 1.0) {
			_state = ADJUSTING;
			return -rel_speed;
		}
      } else if (abs(target->position() - _position)<40000) {
		if ((abs(rel_speed) < 4.5) || (abs(rel_speed) > 5.5)) {
		  rel_speed -= polar(10.0, arg(target->relative_position()));
		  _state = ADJUSTING;
		  return -rel_speed;
		}
      }
  }else {
  }
  
  return complex<double>(0.0,0.0);
}
  
uint32_t satellite::time_to_travel_to(double target_orbit)
{
    return M_PI * sqrt(pow(abs(_position) + target_orbit,3)/(8*MU)) - 1;
  
}

complex<double> satellite::position_at(uint32_t time_step_forward)
{
  if (time_step_forward + _time_step > _trajectoire->known_time_steps()) {
	uint32_t known_time = _trajectoire->known_time_steps();
	for (unsigned int i=known_time; i<= time_step_forward + _time_step; i++) {
	  renderer::getInstance()->unlock();
	  renderer::getInstance()->lock();
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

/*complex<double> stellite::perige_in(uint32_t *time_step_forward) {
  vm_state *clone= vm->clone();
  for (int i=0; i<time_step_forward; i++) {
	clone->step();
  }
}*/