#ifndef RENDERER_H
#define RENDERER_H

#include "common.h"
#include "satellite.h"
#include <vector>

#ifndef START_TIMER
#define START_TIMER 0
#endif

class renderer
{
	private :
		pthread_t _mainthread;
		static renderer *_single_renderer;
		static bool _running;
		void init();
		void terminate();
		void draw(BITMAP* bmp);
		void init_bitmap(BITMAP **bmp);
		
		double _sizeX;
		vector<satellite *> sats;
		pthread_mutex_t _main_mutex;
		vector<double> _radius;
		complex<double> position_to_test;
		double SCALE;
		double _fuel;
		double _max_fuel;
		int m_timer;
		bool fps_toggle;
	public:

		renderer():_sizeX(100.0){ SCALE = 2E3;_fuel = 1; _max_fuel = 1; m_timer = START_TIMER; fps_toggle = false;}
		~renderer(){}

		
		static void * mainLoop(void* params);
		void lock();
		void unlock();
		void add_sat(satellite* sat);
		void add_radius(double radius);
		void add_position(complex<double> position) {position_to_test = position;}
		void set_max_fuel(double max_fuel) {_max_fuel = max_fuel;}
		void set_fuel(double fuel) {_fuel = fuel;}
		int get_timer(){return m_timer;}
		bool get_fps_toggle(){return fps_toggle;}
		static renderer *getInstance ()
		{
			if (NULL == _single_renderer)
			{
				_single_renderer =  new renderer();
				_single_renderer->init();
			}
			
			return _single_renderer;
		}

		static void kill ()
		{
			if (NULL != _single_renderer)
			{
				_single_renderer->terminate();
				delete _single_renderer;
				_single_renderer = NULL;
			}
		}
};

#endif

