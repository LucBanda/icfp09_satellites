#include "common.h"
#include "controller.h"

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
  
  if (time_step == 2) { //ignition when horizontal
	
	
	
	double my_orbit =  sqrt((x * x) + (y*y));
	double target_orbit = vm->output_ports[_target_orbit_addr];
	
	cout << "my_orbit : "<< my_orbit << endl;
	cout << "target_orbit : "<< target_orbit << endl;
	//calculate hohmann transfert function
	double dv_abs = sqrt(MU/my_orbit)*(sqrt(2.0*target_orbit/(target_orbit+my_orbit))-1.0) -0.5;
	//double dv_abs = 5000;
	cout << "dv_abs : "<< dv_abs << endl;
	*dvx = dv_abs * cos(atan(x/y));
	*dvy = dv_abs * sin(atan(x/y));
	cout << "*dvx : "<< *dvx << endl;
	cout << "*dvy : "<< *dvy <<  "," << x << "," << y<< endl;
	
	ignition_time = time_step;
	cout << "ignition time : "<< ignition_time << endl;
	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1-sqrt(2*my_orbit/(target_orbit+my_orbit)));
	
	speed_back_x = dv_abs * cos(atan(x/y) + M_PI) ;
	speed_back_y = dv_abs * sin(atan(x/y) + M_PI) ;
	
	time_to_stop = ignition_time + M_PI * sqrt(pow(my_orbit + target_orbit,3)/(8*MU));
	
	cout << "stop in : " << time_to_stop << endl;
	cout << "ignition" << endl;
	getchar();
  } else {
	if (time_step - time_to_stop == 0) {
	  *dvx = speed_back_x;
	  *dvy = speed_back_y;
	  cout << "stop : " << *dvx << "," << *dvy <<  endl;
	} else {
	  *dvx = 0;
	  *dvy = 0;
	}
	
  }
  
}


bool hohmann::step(uint32_t time_step) {
  cout << "time_step : " << time_step << endl;
  /*if (time_step  ) // init 
  {
	return false;
  } */
  
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
  last_x = vm->output_ports[_vx_addr];
  last_y = vm->output_ports[_vy_addr];
  return false;
}

void hohmann::monitor() {
  	cout << "fuel : "<< vm->output_ports[_fuel_addr] << endl;
	cout << "x : "<< vm->output_ports[_vx_addr] << endl;
	cout << "y : "<< vm->output_ports[_vy_addr] << endl;
	cout << "radius : " << sqrt((vm->output_ports[_vx_addr] * vm->output_ports[_vx_addr]) + (vm->output_ports[_vy_addr] * vm->output_ports[_vy_addr])) << endl;
	cout << "target : " << vm->output_ports[_target_orbit_addr] << endl;
	cout << "score : " << vm->output_ports[_score_addr] << endl;
}