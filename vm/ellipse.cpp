#include "common.h"
#include "ellipse.h"

bool satellipse::is_defined()
{
  return (_state == ST_DEFINED);
}
 
void satellipse::add_position(complex<double> new_position, int time_step){
    int i;
	_trace.push_back(new_position);
	
  
  
}