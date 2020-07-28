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
#define WAYPT_COL al_map_rgb(200, 0, 0)
#define BLACK al_map_rgb(0, 0, 0)
#define TANK_COL al_map_rgb(0, 0, 100)
#define WHITE_COL al_map_rgb(250, 250, 250)
#define TO_SCREEN(c) SCREEN_W / 2. + real(c) / SCALE, SCREEN_H / 2. - imag(c) / SCALE

#define SHAPE_SCALE	2.
const int SCREEN_W = 2000. / SHAPE_SCALE;
const int SCREEN_H = 2000. / SHAPE_SCALE;


using namespace std;

renderer::renderer() {
	SCALE = 2E3;
	draw_decimation = 20;
	FPS = 60 * draw_decimation;
	debug_relative_position = false;
	_target_ellipse = NULL;
	scale_edited = false;
}

double start_radius = 0;
void renderer::draw() {
#ifdef ALLEGRO
	Complex main_sat = _vm->get_absolute_position();
	double fuel = _vm->get_fuel();
	double max_fuel = _vm->get_fuel_max();
	double radius = _vm->get_radius();

	al_draw_filled_rectangle(0, 0, SCREEN_W, SCREEN_H, BACKGROUND_COL);
	al_draw_filled_circle(SCREEN_W / 2., SCREEN_H / 2., 6.357e6 / SCALE,
						  BOULDER_COL);

	al_draw_filled_rectangle(SCREEN_W - 100., SCREEN_H - 110., SCREEN_W - 50.,
							 SCREEN_H - 10., BLACK);
	al_draw_filled_rectangle(SCREEN_W - 98., SCREEN_H - 108., SCREEN_W - 52.,
							 SCREEN_H - 12. - (fuel / max_fuel) * 95.,
							 WHITE_COL);

	if (_vm->get_max_tank_fuel()) {
		al_draw_filled_rectangle(SCREEN_W - 200., SCREEN_H - 110., SCREEN_W - 150.,
								SCREEN_H - 10., BLACK);
		al_draw_filled_rectangle(SCREEN_W - 198., SCREEN_H - 108., SCREEN_W - 152.,
								SCREEN_H - 12. - (_vm->get_tank_fuel() / _vm->get_max_tank_fuel()) * 95. ,
								WHITE_COL);
	}

	if (!scale_edited && abs(main_sat) / SCALE > (double)SCREEN_W / 2.5) {
		SCALE = abs(main_sat) / SCREEN_W * 2.5;
	}
	al_draw_filled_circle(SCREEN_W / 2. + real(main_sat) / (SCALE),
						  SCREEN_H / 2. - imag(main_sat) / (SCALE), 10 / SHAPE_SCALE, ME_COL);

	for (int i = 0; i < _vm->nb_of_targets; i++) {
		if (_vm->is_target_validated(i))
			continue;
		Complex pos = _vm->get_target_absolute_position(i);
		al_draw_filled_circle(SCREEN_W / 2. + real(pos) / (SCALE),
							  SCREEN_H / 2. - imag(pos) / (SCALE), 7.5/ SHAPE_SCALE,
							  SAT_COL);
		if (!scale_edited && abs(pos) / SCALE > (double)SCREEN_W / 2.5) {
			SCALE = abs(pos) / SCREEN_W * 2.5;
		}
	}

	if (!scale_edited && radius / SCALE > (double)SCREEN_W / 2.5) {
		SCALE = radius / SCREEN_W * 2.5;
	}
	al_draw_circle(SCREEN_W / 2., SCREEN_H / 2., radius / SCALE, CRATER_COL,
					3.f/ SHAPE_SCALE);

	al_draw_filled_circle(SCREEN_W / 2. + real(_vm->get_tank_absolute_position()) / (SCALE),
						  SCREEN_H / 2. - imag(_vm->get_tank_absolute_position()) / (SCALE), 10 / SHAPE_SCALE, TANK_COL);

	string text;
	text = to_string(_vm->get_relative_distance(0)) + " m";
	al_draw_text(debug_font, al_map_rgb(0,0,0), 0, 0, ALLEGRO_ALIGN_LEFT, text.c_str());
	text = to_string(_vm->time_step) + " s";
	al_draw_text(debug_font, al_map_rgb(0,0,0), 0, 60, ALLEGRO_ALIGN_LEFT, text.c_str());
#endif
}
void renderer::set_target_ellipse(ellipse *ellip) { _target_ellipse = ellip; }

enum MYKEYS { KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_R };
void renderer::mainLoop(void *params) {
#ifdef ALLEGRO
	ALLEGRO_DISPLAY *display = NULL;
	ALLEGRO_EVENT_QUEUE *event_queue = NULL;
	ALLEGRO_TIMER *timer = NULL;
	bool key[5] = {false, false, false, false, false};
	bool redraw = false;
	bool doexit = false;

	if (!al_init()) {
		fprintf(stderr, "failed to initialize allegro!\n");
		return;
	}

	if (!al_init_primitives_addon()) {
		fprintf(stderr, "failed to initialize primitives!\n");
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
   al_init_font_addon(); // initialize the font addon
   al_init_ttf_addon();// initialize the ttf (True Type Font) addon

   debug_font = al_load_ttf_font("AllegroBT-Regular.otf",72 / SHAPE_SCALE,0 );

	al_set_new_display_flags(ALLEGRO_WINDOWED | ALLEGRO_RESIZABLE);
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
				case ALLEGRO_KEY_R:
					key[KEY_R] = true;
					break;
			}
		} else if (ev.type == ALLEGRO_EVENT_KEY_UP) {
			switch (ev.keyboard.keycode) {
				case ALLEGRO_KEY_UP:
					key[KEY_UP] = false;
					draw_decimation = draw_decimation + draw_decimation / 4.;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_DOWN:
					key[KEY_DOWN] = false;
					if (draw_decimation <= 1) break;
					draw_decimation = draw_decimation - draw_decimation / 4.;
					FPS = 60 * draw_decimation;
					al_set_timer_speed(timer, 1.0 / FPS);
					break;

				case ALLEGRO_KEY_LEFT:
					key[KEY_LEFT] = false;
					SCALE = SCALE - SCALE / 4.;
					scale_edited = true;
					break;

				case ALLEGRO_KEY_RIGHT:
					key[KEY_RIGHT] = false;
					SCALE = SCALE + SCALE / 4.;
					scale_edited = true;
					break;

				case ALLEGRO_KEY_R:
					key[KEY_R] = false;
					debug_relative_position = !debug_relative_position;
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