#ifndef BIN_H
#define BIN_H
#include "vm_state.h"

class bin_1 : public vm_state {
   public:
	bin_1(int instance);
	virtual ~bin_1();
	void step();
};

class bin_2 : public vm_state {
   public:
	int pos_target_x_addr;
	int pos_target_y_addr;

	bin_2(int instance);
	virtual ~bin_2();
	void step();
	vector<Complex> calculate_targets();
};

class bin_3 : public vm_state {
   public:
	int pos_target_x_addr;
	int pos_target_y_addr;

	bin_3(int instance);
	virtual ~bin_3();
	void step();
	vector<Complex> calculate_targets();
};

class bin_4 : public vm_state {
   public:
	vector<int> pos_target_x_addrs;
	vector<int> pos_target_y_addrs;
	vector<Complex> old_targets;
	vector<Complex> speed_targets;

	bin_4(int instance);
	virtual ~bin_4();
	void step();
	vector<Complex> calculate_targets();
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