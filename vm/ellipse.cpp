#include "common.h"
#include "ellipse.h"
#include "vm_state.h"

void satellipse::add_position(complex<double> new_position, int time_step){
  if (_trace.size() > time_step) return;
	_trace.push_back(new_position); 
}

unsigned int satellipse::known_time_steps() {
  return _trace.size()+1;
}

complex<double> satellipse::get_pos_at(int time_step) {
  return _trace[time_step - 1];
}
