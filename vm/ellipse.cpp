#include "common.h"
#include "ellipse.h"

bool satellipse::is_defined()
{
  return (_state == ST_DEFINED);
}
 
void satellipse::add_position(complex<double> new_position, int time_step){
    int i;
  if (!is_defined()) {
	_trace.push_back(new_position);
	if (time_step > 10) {
	  if (abs(new_position - _trace[0]) < abs(_trace[1] - _trace[0])) { //period lock only one time
		_period = time_step - last_time_orig;
		
		cout << _apogee << " , " << _perige << " , " << _trace[0] << endl;
		for (i = 0; i < _period; i++) {
		  if (abs(_apogee) < abs(_trace[i]))
			_apogee = _trace[i];
		  
		  if (abs(_perige) > abs(_trace[i]))
			_perige = _trace[i];
		}
		_great_axe = _apogee - _perige;
		cout << _apogee << " , " << _perige << " , " << _trace[0] << endl;
		getchar();
		_center = polar(abs(_great_axe) /2, arg(_great_axe));
		
		for (i=0; i<_period/2; i++) {
		  if (abs(arg(_trace[i] - _center)-arg(_great_axe)) - M_PI/2 < 0)
			break;
		}
		complex<double> pos1_of_little_axe = _trace[i-1];
		for (i=_period/2; i<_period; i++) {
		  if (abs(arg(_trace[i] - _center)-arg(_apogee - _perige)) -M_PI/2 > 0)
			break;
		}
		_little_axe = pos1_of_little_axe - _trace[i-1];
		  
		last_time_orig = time_step;
		_trace[0] = new_position;
		_state = ST_DEFINED;
	  }
	}
  } else {
	_trace[time_step - last_time_orig] = new_position;
	if (time_step == _period)
	  last_time_orig = time_step;
  }
  
  
}

complex<double> satellipse::position_in(complex<double> current_position, int current_time_step, int step_forward)
{
  return _trace[current_time_step + step_forward - last_time_orig]; // dangerous : last orbit point
}