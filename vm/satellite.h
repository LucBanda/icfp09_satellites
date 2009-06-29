#ifndef SATELLITE_H
#define SATELLITE_H

#include "common.h"
#include "vm_state.h"
#include <complex>

#define ORBITING				0
#define ADJUSTING				1
#define TRAVELLING				2

class satellite {
  private :
	satellite *_main;
	address addr_x;
	address addr_y;
	complex<double> _relative_position;
	complex<double> _position;
	complex<double> _old_position;
	complex<double> _speed;
	complex<double> _speed_back;
	
	double _orbit;
	int _state;
	uint32_t _ignition_time;
	uint32_t _time_step;
	uint32_t _stop_time;
	
  public:
	satellite(address x, address y, satellite *me=NULL) : _main(me), addr_x(x), addr_y(y), _relative_position(vm->output_ports[x],vm->output_ports[y]), _old_position(0,0), _speed(-1,-1),  _state(ORBITING)
	{
	  if (me == NULL) {
		_position = -_relative_position;
	  } else {
		_position = _relative_position + me->position();
	  }
	  _orbit = abs(_position);
	  _time_step = 0;
	  _stop_time = 0;
	  
	};
	
	complex<double> position(){return _position;}
	complex<double> speed(){return _speed;}
	complex<double> relative_position(){return _relative_position;}
	complex<double> state(){return _state;}
	
	void update(uint32_t time_step);
	
	complex<double> travel_to(double target_orbit, complex<double> *target_position = NULL, bool simulate = false);
	complex<double> travel_to(complex<double> target_position);
	uint32_t time_to_travel_to(double target_orbit);
	complex<double> position_at(uint32_t time_step_forward);
	
};

#endif
