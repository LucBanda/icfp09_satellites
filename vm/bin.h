#ifndef BIN_H
#define BIN_H
#include "vm_state.h"

class bin_1 :public vm_state {
  public:
	bin_1(int instance);
	virtual ~bin_1();
	void step();
	double get_radius();
};

class bin_2 :public vm_state {
  public:
	int pos_target_x_addr;
	int pos_target_y_addr;

	bin_2(int instance);
	virtual ~bin_2();
	//double get_radius();
	void step();
	vector<satellite> get_targets();
};

class bin_3 :public vm_state {
  public:
  	int pos_target_x_addr;
	int pos_target_y_addr;

	bin_3(int instance);
	virtual ~bin_3();
	void step();
	vector<satellite> get_targets();
};

class bin_4 :public vm_state {
  public:
  	vector<int> pos_target_x_addrs;
	vector<int> pos_target_y_addrs;

	bin_4(int instance);
	virtual ~bin_4();
	void step();
	vector<satellite> get_targets();
};

inline vm_state *bin_factory(int instance)
{
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