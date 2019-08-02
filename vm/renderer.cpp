#include <complex>
#include "common.h"

#include <allegro5/allegro_primitives.h>
#include "renderer.h"
#include "vm_state.h"

#define MAP_RES 3000

#define BOULDER_COL al_map_rgb(100, 100, 100)
#define BACKGROUND_COL al_map_rgb(200, 200, 200)
#define CRATER_COL al_map_rgb(200, 100, 100)
#define ME_COL al_map_rgb(0, 200, 0)
#define SAT_COL al_map_rgb(200, 0, 0)
#define ROVER_COL HOME_COL
#define WAYPT_COL al_map_rgb(200, 0, 0)
#define BLACK al_map_rgb(0, 0, 0)

const int SCREEN_W = 2000;
const int SCREEN_H = 2000;

using namespace std;

renderer::renderer() {
	SCALE = 2E3;
	_fuel = 1;
	_max_fuel = 1;
	draw_decimation = 20;
	FPS = 60 * draw_decimation;
}

double start_radius = 0;
void renderer::draw() {
#ifdef ALLEGRO
	al_draw_filled_rectangle(0, 0, SCREEN_W, SCREEN_H, BACKGROUND_COL);
	al_draw_filled_circle(SCREEN_W / 2., SCREEN_H / 2., 6.357e6 / SCALE,
						  BOULDER_COL);
	// al_draw_filled_circle(500 + 0.5f + real(position_to_test)/SCALE,500+ 0.5f
	// - imag(position_to_test)/SCALE, 3, BOULDER_COL);

	al_draw_filled_rectangle(SCREEN_W - 100., SCREEN_H - 100., SCREEN_W - 50.,
							 SCREEN_H - 10., BLACK);
	al_draw_filled_rectangle(SCREEN_W - 98., SCREEN_H - 98., SCREEN_W - 52.,
							 SCREEN_H - 98. + (1 - (_fuel / _max_fuel)) * 100,
							 BACKGROUND_COL);

	if (abs(main_sat) /
			SCALE >
		(double)SCREEN_W / 2.5) {
		SCALE = abs(main_sat) /
				SCREEN_W * 2.5;
	}
	al_draw_filled_circle(SCREEN_W / 2. + real(main_sat) / (SCALE),
						  SCREEN_H / 2. - imag(main_sat) / (SCALE), 10, ME_COL);

	for (vector<Complex>::iterator it = sats.begin(); it != sats.end();
		 it++) {
		al_draw_filled_circle(SCREEN_W / 2. + real(*it) / (SCALE),
							  SCREEN_H / 2. - imag(*it) / (SCALE), 7.5,
							  SAT_COL);
		if (abs(*it) / SCALE >
			(double)SCREEN_W / 2.5) {
			SCALE = abs(*it) /
					SCREEN_W * 2.5;
		}
	}

	for (vector<double>::iterator it = _radius.begin(); it != _radius.end();
		 it++) {
		if (*it / SCALE > (double)SCREEN_W / 2.5) {
			SCALE = *it / SCREEN_W * 2.5;
		}
		al_draw_circle(SCREEN_W / 2., SCREEN_H / 2., *it / SCALE, CRATER_COL,
					   3.f);
	}
#endif
}

void renderer::set_sat(vector<Complex> sat) { sats = sat; }
void renderer::add_radius(double radius) { _radius.push_back(radius); }

enum MYKEYS { KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT };
void renderer::mainLoop(void *params) {
#ifdef ALLEGRO
	ALLEGRO_DISPLAY *display = NULL;
	ALLEGRO_EVENT_QUEUE *event_queue = NULL;
	ALLEGRO_TIMER *timer = NULL;
	bool key[4] = {false, false, false, false};
	bool redraw = false;
	bool doexit = false;

	if (!al_init()) {
		fprintf(stderr, "failed to initialize allegro!\n");
		return;
	}

	if (!al_install_keyboard()) {
		fprintf(stderr, "failed to initialize the keyboard!\n");
		return;
	}

	timer = al_create_timer(1.0 / FPS);
	if (!timer) {
		fprintf(stderr, "failed to create timer!\n");
		return;
	}

	al_set_new_display_flags(ALLEGRO_WINDOWED|ALLEGRO_RESIZABLE);
	display = al_create_display(SCREEN_W, SCREEN_H);
	if (!display) {
		fprintf(stderr, "failed to create display!\n");
		al_destroy_timer(timer);
		return;
	}


	al_clear_to_color(al_map_rgb(255, 0, 255));

	al_set_target_bitmap(al_get_backbuffer(display));

	event_queue = al_create_event_queue();
	if (!event_queue) {
		fprintf(stderr, "failed to create event_queue!\n");
		al_destroy_display(display);
		al_destroy_timer(timer);
		return;
	}

	al_register_event_source(event_queue, al_get_display_event_source(display));

	al_register_event_source(event_queue, al_get_timer_event_source(timer));

	al_register_event_source(event_queue, al_get_keyboard_event_source());

	al_clear_to_color(al_map_rgb(0, 0, 0));

	al_flip_display();

	al_start_timer(timer);

	int i = 0;

	while (!doexit) {
		ALLEGRO_EVENT ev;
		al_wait_for_event(event_queue, &ev);

		if (ev.type == ALLEGRO_EVENT_TIMER) {
			doexit = idle(idle_param);
			i += 1;
			if (i > draw_decimation) {
				redraw = true;
				i = 0;
			}
		} else if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
			break;
		} else if (ev.type == ALLEGRO_EVENT_KEY_DOWN) {
			switch (ev.keyboard.keycode) {
				case ALLEGRO_KEY_UP:
					key[KEY_UP] = true;
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = true;
					break;

				case ALLEGRO_KEY_LEFT:
					key[KEY_LEFT] = true;
					break;

				case ALLEGRO_KEY_RIGHT:
					key[KEY_RIGHT] = true;
					break;
			}
		} else if (ev.type == ALLEGRO_EVENT_KEY_UP) {
			switch (ev.keyboard.keycode) {
				case ALLEGRO_KEY_UP:
					key[KEY_UP] = false;
					draw_decimation = draw_decimation + draw_decimation / 4;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = false;
					if (draw_decimation <= 6) break;
					draw_decimation = draw_decimation - draw_decimation / 4;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_LEFT:
					key[KEY_LEFT] = false;
					break;

				case ALLEGRO_KEY_RIGHT:
					key[KEY_RIGHT] = false;
					break;

				case ALLEGRO_KEY_ESCAPE:
					doexit = true;
					break;
			}
		} else {
		}

		if (redraw && al_is_event_queue_empty(event_queue)) {
			redraw = false;

			al_clear_to_color(al_map_rgb(0, 0, 0));
			draw();

			al_flip_display();
		}
	}

	al_destroy_timer(timer);
	al_destroy_display(display);
	al_destroy_event_queue(event_queue);

#endif
	return;
}