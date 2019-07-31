#ifndef RENDERER_H
#define RENDERER_H

#include <math.h>
#include <vector>
#include "common.h"
#include "vm_state.h"

#ifndef START_TIMER
#define START_TIMER 0
#endif

class renderer {
   private:
	static bool _running;
	void draw();

	vector<double> _radius;
	double SCALE;
	double _fuel;
	double _max_fuel;
	int m_timer;
	bool fps_toggle;
	int FPS;
	int draw_decimation;

   public:
	std::function<void(void *)> idle;
	void *idle_param;

	satellite main_sat;
	vector<satellite> sats;

	renderer();
	~renderer() {}

	void set_sat(vector<satellite> sat);
	void *mainLoop(void *params);
	void add_radius(double radius);
	void set_max_fuel(double max_fuel) { _max_fuel = max_fuel; }
	void set_fuel(double fuel) { _fuel = fuel; }
	int get_timer() { return m_timer; }
	bool get_fps_toggle() { return fps_toggle; }
};

#endif
