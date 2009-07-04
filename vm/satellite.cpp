#include "common.h"
#include "satellite.h"


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
  
  if (_main == NULL)
	_position = -_relative_position;
  else
	_position = _main->position() + _relative_position;
  _time_step = time_step;
  _speed = _position - _old_position;
  _orbit = abs(_position);
  if(_main)
	cout <<"target orbit " << _orbit << endl;
  else
	cout <<"my orbit " << _orbit << endl;
  
  _angular_speed = to_range(arg(_position) - arg(_old_position));
  
  _old_position = _position;
  if (_trajectoire != NULL) {
	_trajectoire->add_position(_position, time_step);
  }
  
}

complex<double> satellite::travel_to(double target_orbit, complex<double> *target_position, bool simulate)
{
  complex<double> delta_v(0.0,0.0);
  if ((_state == ORBITING) || (simulate)) {

	double dv_abs = sqrt(MU/_orbit)*(sqrt(2.0*target_orbit/(target_orbit+_orbit))-1.0);
	
	delta_v =  polar(dv_abs, arg(_speed));
	_ignition_time = _time_step;

	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1.0-sqrt(2*_orbit/(target_orbit+_orbit)));
	
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
  cout <<"state : " <<  _state << endl;
  if (_state == ORBITING) {
	uint32_t arrival_time = time_to_travel_to(target->orbit());
	complex<double> position_to_arrive;
    complex<double> needed_delta_v = travel_to(target->orbit(), &position_to_arrive, true);
	cout << target->orbit() << endl;
	/*if (abs(position_to_arrive - target->position_at(arrival_time)) < 10000.0)
	  getchar();*/
	if (abs(position_to_arrive - target->position_at(arrival_time+1)) < 15000.0) {
	  _state = TRAVELLING; //validate simulation
	  return needed_delta_v;
	}
  } else if (_state == TRAVELLING){
	complex<double> action = travel_to(target->orbit());
	if (_state == ORBITING)
	  _state = DOCKING;
	return action;
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
      } else if (abs(target->position() - _position)<20000) {
		if ((abs(rel_speed) < 4.5) || (abs(rel_speed) > 5.5)) {
		  rel_speed -= polar(5.0, arg(target->relative_position()));
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
    return M_PI * sqrt(pow(_orbit + target_orbit,3)/(8*MU)) - 1;
  
}

complex<double> satellite::position_at(uint32_t time_step_forward)
{
  vm_state *clone= vm->clone();
  for (int i<0; i<time_step_forward; i++) {
	clone->step();
  }
  return polar(_orbit, arg(_position) + (time_step_forward+1) * (_angular_speed)) ;
}