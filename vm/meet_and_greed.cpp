#include "common.h"
#include "meet_and_greed.h"

meetandgreed::meetandgreed(trace_generator *trace, double instance):Icontroller(trace, instance) {
  _score_addr = 0x0;
  _fuel_addr = 0x1;
  _vx_addr = 0x2;
  _vy_addr = 0x3;
  _sat_x_addr = 0x4;
  _sat_y_addr = 0x5;
  
  _delta_vx_addr = 0x2;
  _delta_vy_addr = 0x3;
  _instance_addr = 0x3E80;
  vm->input_ports[_instance_addr] = instance;
  _trace->add_command(0, _instance_addr, _instance, vm->output_ports[_score_addr]);
  
  max_angular_speed = 0;
  time_to_stop = -1;
  ignition_time = -1;
  delay = -1;
  rectified = false;
}



void meetandgreed::calculate_action(double *dvx, double *dvy, uint32_t time_step) {
  cout << "\x1b[2J\x1b[H";
  complex<double> my_abs_pos(-vm->output_ports[_vx_addr], -vm->output_ports[_vy_addr]);
  
  complex<double> sat_abs_pos = +(my_abs_pos + complex<double>(vm->output_ports[_sat_x_addr], vm->output_ports[_sat_y_addr]));
  double sat_abs_theta = arg(sat_abs_pos) ;
  
  double my_orbit = abs(my_abs_pos);
  double target_orbit = abs(sat_abs_pos) - (_instance == 2002) * 200;

  if (rectified) {
      *dvx = 0;
      *dvy = 0;
      old_my_abs_pos = my_abs_pos;
      old_sat_abs_pos = sat_abs_pos;
      rectified = false;
      return;
  }
  if (sat_abs_theta * arg(old_sat_abs_pos) < 0) //angle just changed sign
  {
	old_sat_abs_pos = sat_abs_pos;
	return;
  }
  double angular_speed = sat_abs_theta - arg(old_sat_abs_pos);
  cout << "sat polar coord:( " << abs(sat_abs_pos) << " , "<< sat_abs_theta<<" ) "<< endl;
  cout << "sat position " << real(sat_abs_pos) << " , " << imag(sat_abs_pos) << endl;
  cout << "angular speed of sat " << angular_speed << endl;
  
  
  double arriving_angle = arg(-my_abs_pos);
  double time_to_arrive = M_PI * sqrt(pow(my_orbit + target_orbit,3)/(8*MU)) - 1;
  
  cout << "arriving angle " << arriving_angle << endl;
  
  int32_t fudge = 0;
  if ((_instance == 2001) || (_instance == 2004))
	fudge = +1;
  else if (_instance == 2003)
	fudge = 1;
  double sat_arrival_angle = sat_abs_theta + (time_to_arrive + fudge) * (angular_speed) ;
  while(sat_arrival_angle > M_PI) sat_arrival_angle -=2*M_PI;
  while(sat_arrival_angle < -M_PI) sat_arrival_angle +=2*M_PI;
  
  cout << "arrival of sat angle " << sat_arrival_angle<< endl;
  
  bool meeting_point_conditions = (arriving_angle < (sat_arrival_angle + 0.0004))
							   && (arriving_angle > (sat_arrival_angle - 0.0004));
  
  cout << "angle " << arg(sat_abs_pos - my_abs_pos) << endl;
  
  if ((meeting_point_conditions) && (time_step != 0) && (ignition_time == -1)){
	cout << "*** ignition ***" << endl;
	
	//ignition_time
	double dv_abs = sqrt(MU/my_orbit)*(sqrt(2.0*target_orbit/(target_orbit+my_orbit))-1.0);
	
	complex<double> speed = polar(dv_abs, arg(my_abs_pos) - M_PI/2);
	*dvx = real(speed);
	*dvy = imag(speed);
	
	ignition_time = time_step;
	//vitesse back
	dv_abs = sqrt(MU/target_orbit)*(1.0-sqrt(2*my_orbit/(target_orbit+my_orbit)));
	
	speed = polar(dv_abs, arg(my_abs_pos) + M_PI/2);
	
	speed_back_x = real(speed) ;
	speed_back_y = imag(speed) ;
	
	time_to_stop = ignition_time +  M_PI * sqrt(pow(my_orbit + target_orbit,3)/(8*MU));
	
  } else if (time_to_stop == 1+time_step+(_instance == 2002)*( - 14) + (_instance == 2003)*(-4)) {
	  *dvx = speed_back_x ;
	  *dvy = speed_back_y ;
      time_to_stop = -1;
  /*} else if ((time_to_stop == -1) && (ignition_time != -1)){
      //vitesse relative
      complex< double> rel_speed = my_abs_pos - old_my_abs_pos - sat_abs_pos + old_sat_abs_pos;
      if (abs(sat_abs_pos - my_abs_pos)<1000) {
          if (abs(rel_speed) > 100) {
              rel_speed += polar(1.0, arg(-sat_abs_pos + my_abs_pos));
              *dvx = real(-rel_speed);
              *dvy = imag(-rel_speed);
              cout << "setting same speed " << -rel_speed <<","<<abs(rel_speed) <<endl;
              rectified = true;
          } else {
              *dvx = 0;
              *dvy = 0;
          }
      } /*else if (abs(sat_abs_pos - my_abs_pos)<10000) {
            rel_speed += polar(100.0, arg(sat_abs_pos - my_abs_pos));
            *dvx = real(-rel_speed);
            *dvy = imag(-rel_speed);
            cout << "setting same speed " << -rel_speed <<","<<abs(rel_speed) <<endl;
            rectified = true;
            getchar();
      }*/
  }else {
      *dvx = 0;
      *dvy = 0;
  }
  old_my_abs_pos = my_abs_pos;
  old_sat_abs_pos = sat_abs_pos;
}


bool meetandgreed::step(uint32_t time_step) {
  
  if (vm->output_ports[_score_addr]) {
	_trace->add_command(time_step, 0, 0, vm->output_ports[_score_addr]);
    return true;
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

void meetandgreed::monitor() {
  	cout << "fuel : "<< vm->output_ports[_fuel_addr] << endl;
	cout << "abs position" << -complex<double>(vm->output_ports[_vx_addr],  vm->output_ports[_vy_addr]) << endl;
	cout << "radius : " << sqrt((vm->output_ports[_vx_addr] * vm->output_ports[_vx_addr]) + (vm->output_ports[_vy_addr] * vm->output_ports[_vy_addr])) << endl;
	cout << "relative distance to target : " << sqrt(pow(vm->output_ports[_sat_x_addr],2) + pow(vm->output_ports[_sat_y_addr],2)) << endl;
	cout << "score : " << vm->output_ports[_score_addr] << endl;
	usleep(0);
}