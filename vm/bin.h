#ifndef BIN_H
#define BIN_H
#include "vm_state.h"

class bin_1 : public vm_state {
   public:
	bin_1(int instance);
	virtual ~bin_1();
	void step();
	double get_relative_distance(int target);
	Complex get_absolute_position();
	Complex get_target_absolute_position(int target);
	double get_relative_delta_speed(int target);
	Complex get_relative_speed(int target);

	private:
	double old_pos_x;
	double old_pos_y;
	double speed_x;
	double speed_y;
};

class bin_2 : public vm_state {
   public:

	bin_2(int instance);
	virtual ~bin_2();
	void step();
	vector<Complex> calculate_targets();
	double get_relative_distance(int target);
	Complex get_absolute_position();
	Complex get_target_absolute_position(int target);
	double get_relative_delta_speed(int target);
	Complex get_relative_speed(int target);

	private:
	int pos_target_x_addr;
	int pos_target_y_addr;
	double old_target_rel_pos_x;
	double old_target_rel_pos_y;
	double rel_speed_targets_x;
	double rel_speed_targets_y;
};

class bin_3 : public vm_state {
   public:
	bin_3(int instance);
	virtual ~bin_3();
	void step();
	vector<Complex> calculate_targets();
	double get_relative_distance(int target);
	Complex get_absolute_position();
	Complex get_target_absolute_position(int target);
	double get_relative_delta_speed(int target);
	Complex get_relative_speed(int target);

	private:
	int pos_target_x_addr;
	int pos_target_y_addr;
	double old_target_rel_pos_x;
	double old_target_rel_pos_y;
	double rel_speed_targets_x;
	double rel_speed_targets_y;
};

class bin_4 : public vm_state {
   public:
	bin_4(int instance);
	virtual ~bin_4();
	void step();
	double get_relative_distance(int target);
	Complex get_absolute_position();
	Complex get_target_absolute_position(int target);
	double get_relative_delta_speed(int target);
	Complex get_relative_speed(int target);
	double get_relative_distance_to_tank();
	Complex get_tank_absolute_position();
	double get_tank_fuel();
	double get_max_tank_fuel();
	bool is_target_validated(int target);

	private:
	vector<int> pos_target_x_addrs;
	vector<int> pos_target_y_addrs;
	vector<double> old_target_rel_pos_x;
	vector<double> old_target_rel_pos_y;
	vector<double> rel_speed_targets_x;
	vector<double> rel_speed_targets_y;
	vector<bool> validated_target;
	address tank_x_addr;
	address tank_y_addr;
	address tank_fuel_addr;
	vector<int> validated_target_addr;

};

inline vm_state *bin_factory(int instance) {
	vm_state *ret = NULL;
	if ((instance < 1005) && (instance > 1000))
		ret = new bin_1(instance);
	else if ((instance < 2005) && (instance > 2000))
		ret = new bin_2(instance);
	else if ((instance < 3005) && (instance > 3000))
		ret = new bin_3(instance);
	else if ((instance < 4005) && (instance > 4000))
		ret = new bin_4(instance);

	return ret;
}

#endif