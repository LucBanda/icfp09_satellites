#ifndef RENDERER_H
#define RENDERER_H

#include <math.h>
#include <vector>
#include "common.h"
#include "vm_state.h"
#include "allegro5/allegro.h"
#include "allegro5/allegro_font.h"
#include "allegro5/allegro_ttf.h"
#include "ellipse.h"

#ifndef START_TIMER
#define START_TIMER 0
#endif

class renderer {
   private:
	static bool _running;
	void draw();

	double SCALE;
	bool scale_edited;
	int m_timer;
	bool fps_toggle;
	int FPS;
	float draw_decimation;
	bool debug_relative_position;
	ALLEGRO_FONT *debug_font;
	ellipse *_target_ellipse;
	vm_state *_vm;

   public:
	std::function<bool(void *)> idle;
	void *idle_param;

	renderer();
	~renderer() {}
	void set_vm(vm_state *vm) {_vm = vm;}
	void set_target_ellipse(ellipse *target_ellipse);
	void mainLoop(void *params);
	int get_timer() { return m_timer; }
	bool get_fps_toggle() { return fps_toggle; }
};

#endif
