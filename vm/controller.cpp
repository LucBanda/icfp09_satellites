#include "common.h"
#include "controller.h"
#include <complex>

hohmann::hohmann(trace_generator *trace, double instance):Icontroller(trace, instance) {
  _score_addr = 0x0;
  _fuel_addr = 0x1;
  _vx_addr = 0x2;
  _vy_addr = 0x3;
  _target_orbit_addr = 0x4;
  _instance_addr = 0x3E80;
  _delta_vx_addr = 0x2;
  _delta_vy_addr = 0x3;
  	vm->input_ports[_instance_addr] = instance;
  _trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  ignition_time = 0;
}



void hohmann::calculate_action(double *dvx, double *dvy, uint32_t time_step) {
  
  double x=vm->output_ports[_vx_addr];
  double y=vm->output_ports[_vy_addr];
  complex<double> my_abs_pos(-x, -y);
  if (time_step == 1) { //ignition asap
	
	double my_orbit =  sqrt((x * x) + (y*y));
	double target_orbit = vm->output_ports[_target_orbit_addr];
	
	//calculate hohmann transfert function
	double dv_abs = sqrt(MU/my_orbit)*(sqrt(2.0*target_orbit/(target_orbit+my_orbit))-1.0);
	
	complex<double> speed;
	if (_instance == 1004)
	  speed = polar(dv_abs, arg(my_abs_pos) + M_PI/2); //this should be +M_PI/2 for the 1004 as it is couterclock wise
	else 
	  speed = polar(dv_abs, arg(my_abs_pos) - M_PI/2); //this should be +M_PI/2 for the 1004 as it is couterclock wise
	*dvx = real(speed);
	*dvy = imag(speed);
	
	ignition_time = time_step;
	
	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1-sqrt(2*my_orbit/(target_orbit+my_orbit)));
	if (_instance == 1004)
	  speed = polar(dv_abs, arg(my_abs_pos) - M_PI/2);
	else
	  speed = polar(dv_abs, arg(my_abs_pos) + M_PI/2);
	speed_back_x = real(speed);
	speed_back_y = imag(speed);
	
	
	time_to_stop = ignition_time + M_PI * sqrt(pow(my_orbit + target_orbit,3)/(8*MU));
	
  } else {
	if (time_step - time_to_stop == 0) {
	  *dvx = speed_back_x;
	  *dvy = speed_back_y;
	} else {
	  *dvx = 0;
	  *dvy = 0;
	}
	
  }
  
}


bool hohmann::step(uint32_t time_step) {
  
  if (vm->output_ports[_score_addr]) {
	return _trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
  }
  
  double dvx, dvy;
  calculate_action(&dvx, &dvy, time_step);
  
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

void hohmann::monitor() {
  	/*cout << "fuel : "<< vm->output_ports[_fuel_addr] << endl;
	cout << "x : "<< vm->output_ports[_vx_addr] << endl;
	cout << "y : "<< vm->output_ports[_vy_addr] << endl;
	cout << "radius : " << sqrt((vm->output_ports[_vx_addr] * vm->output_ports[_vx_addr]) + (vm->output_ports[_vy_addr] * vm->output_ports[_vy_addr])) << endl;
	cout << "target : " << vm->output_ports[_target_orbit_addr] << endl;*/
	if (vm->output_ports[_score_addr])
	  cout << "score : " << vm->output_ports[_score_addr] << endl;
}