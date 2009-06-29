#include "common.h"
#include "satellite.h"

void satellite::update(uint32_t time_step)
{
  _relative_position = complex<double>(vm->output_ports[addr_x], vm->output_ports[addr_y]);
  
  if (_main == NULL)
	_position = -_relative_position;
  else
	_position = _main->position() + _relative_position;
  _time_step = time_step;
  _speed = _position - _old_position;
  _orbit = abs(_position);
  
  
  _old_position = _position;
  
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
	_stop_time = _ignition_time +  time_to_travel_to(target_orbit);
	
	if (!simulate)
	  _state = TRAVELLING;
  cout << "travel_to " << delta_v <<endl;
  } else if (_state == TRAVELLING) {
	if (_stop_time == _time_step) {
	 _state = ORBITING;
	 delta_v = _speed_back;
	}
  }
  return delta_v;
}

/*complex<double> satellite::meet(satellite *target)
{
  if (_state == ORBITING) {
	uint32_t arrival_time = time_step + me->time_to_travel_to(target->orbit());
	complex<double> position_to_arrive;
    complex<double> needed_delta_v = me->travel_to(target->orbit(), &position_to_arrive);
	if (abs(position_to_arrive - target->position_at(arrival_time)) < 1000.0) {
	  me->_state = TRAVELLING;
	  return needed_delta_v;
	}
  }
}*/
  
uint32_t satellite::time_to_travel_to(double target_orbit)
{
    return M_PI * sqrt(pow(_orbit + target_orbit,3)/(8*MU)) - 1;
  
}

complex<double> satellite::position_at(uint32_t time_step_forward)
{
  
}