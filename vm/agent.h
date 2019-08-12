#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "vm_state.h"

typedef std::map<int, Complex> executionT;

enum flying_state {
	FS_ORBIT,
	FS_FLY,
	FS_STICK_TO_TARGET,
	FS_ON_TARGET_ORBIT,
	FS_FLYING_TO_TARGET,
	FS_LOST,
	FS_TANKING
};

class agent {
   public:
	vm_state *vm;
	double max_time_step = 750000;
	int last_validated_time;

	agent(int instance);
	virtual ~agent();

	virtual double get_score() = 0;
	virtual bool stick_to_target() = 0;
	virtual void step();
	virtual void set_execution_map(executionT *map);
	virtual double run();
	vector<int> non_validated_targets;

   protected:
	executionT execution_map;
	enum flying_state fly_state;
	int last_time_fs_changed;
	int time_close_to_orbit;
	double distance_when_crossed;
	double abs_delta_v_when_crossed;

};

class agent1 : public agent {
   public:
	agent1(int instance);
	~agent1();

	bool stick_to_target();
	double get_score();

};

class agent2 : public agent {
   public:
	agent2(int instance);
	~agent2();

	bool stick_to_target();
	double get_score();

};

class agent3 : public agent {
   public:
	agent3(int instance);
	~agent3();

	bool stick_to_target();
	double get_score();
};


class agent4 : public agent {
   public:
	agent4(int instance);
	~agent4();

	bool stick_to_target();
	double get_score();

	private:
	void update_status();
	double get_intermediate_score();
	vector<int> validated_targets;
	vector<int> validated_time_steps;
	double distance_when_lost;
	double max_distance;
	Complex relative_speed_to_tank_when_crossed;
};

static inline agent *agent_factory(int instance) {
	int pb = instance /1000;
	switch (pb) {
	case 1:
		return new agent1(instance);
	case 2:
		return new agent2(instance);
	case 3:
		return new agent3(instance);
	case 4:
		return new agent4(instance);
	default:
		cout << instance << " not allowed" << endl;
	}
	return NULL;
}

#endif