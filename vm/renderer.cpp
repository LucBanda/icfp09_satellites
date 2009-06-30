#include "common.h"
#include <complex>

#include "renderer.h"

#include "vm_state.h"


#define ECRAN_X	1050
#define ECRAN_Y	800
#define MODE    GFX_AUTODETECT_WINDOWED

#define MAP_RES 3000

#define BOULDER_COL  	makecol(100,100,100)
#define BACKGROUND_COL 	makecol(200,200,200)
#define CRATER_COL	makecol(200,100,100)
#define ME_COL	makecol(0,200,0)
#define SAT_COL	makecol(200,0,0)
#define ROVER_COL	HOME_COL
#define WAYPT_COL	makecol(200,0,0)


#define ALLEGRO


using namespace std;
renderer *renderer::_single_renderer = NULL;
bool renderer::_running = false;

double SCALE = 2E3;

void renderer::add_sat(satellite *sat) {
  sats.push_back(sat);
}

double start_radius = 0;
void renderer::draw(BITMAP* bmp)
{
#ifdef ALLEGRO
	lock();

	rectfill(bmp, 0,0,1000,1000, BACKGROUND_COL);
	
	
	for (vector<satellite *>::iterator it = sats.begin(); it != sats.end(); it++) {
	  int color;
	  if ((*it)->main_sat()) {
		color = ME_COL;
		if (start_radius ==0)
		  start_radius = (*it)->orbit();
	  } else {
		color = SAT_COL;
	  }
	  if ((*it)->main_sat())
		circle(bmp, 500,500, start_radius/SCALE, CRATER_COL);
	  else
		circle(bmp, 500,500, abs((*it)->orbit())/SCALE, CRATER_COL);
	  circlefill(bmp, 500+real((*it)->position())/(SCALE),  500-imag((*it)->position())/(SCALE) , 4, color);
	  
	  if ((*it)->orbit() > SCALE*490)
		SCALE = (*it)->orbit()/490;
	}
	
	
	
	complex<double> my_abs_pos(-vm->output_ports[0x2], -vm->output_ports[0x3]);
	
	

	
	if (start_radius == 0)
	  start_radius = abs(my_abs_pos);
	
	complex<double> sat_abs_pos = +(my_abs_pos + complex<double>(vm->output_ports[0x4], vm->output_ports[0x5]));
	
	circlefill(bmp, 500+real(sat_abs_pos) / (SCALE),  500-imag(sat_abs_pos) / (SCALE) , 4, SAT_COL);
	
	
	
	unlock();
#endif
}

void renderer::init_bitmap(BITMAP **bmp)
{
#ifdef ALLEGRO
	lock();
	if ((*bmp) == NULL) {
		*bmp = create_bitmap(1000,1000);
		_sizeX = 1000;
	} 
	unlock();
#endif
}

 
void * renderer::mainLoop(void* params)
{
#ifdef ALLEGRO
	allegro_init();             /* initialise the Allegro library */

	install_mouse();
	if (set_gfx_mode(MODE,ECRAN_X,ECRAN_Y,0,0)!=0) {

		return NULL;
	}

	show_mouse(screen);
	BITMAP *double_buffer = NULL;
	enable_hardware_cursor();
	clear_bitmap(screen);
	
	
	while (_running){ 
		usleep(100000);
		_single_renderer->init_bitmap(&double_buffer);	
		_single_renderer->draw(double_buffer);
		stretch_blit(double_buffer, screen, 0,0,double_buffer->w,double_buffer->h,0,0,ECRAN_Y,ECRAN_Y);
		
	}
	allegro_exit();

#endif
	return 0;

}


void renderer::init()
{
#ifdef ALLEGRO
	_running = true;
	pthread_create(&_mainthread, NULL, renderer::mainLoop, NULL);;
	pthread_mutex_init(&_map_mutex, NULL);
#endif
}
				 // Fonctions de création et destruction du singleton
void renderer::terminate()
{
#ifdef ALLEGRO
	_running =false;
	pthread_mutex_destroy(&_map_mutex);
	pthread_join(_mainthread, NULL);
#endif
}


void renderer::lock()
{
	pthread_mutex_lock(&_map_mutex);
}
void renderer::unlock()
{
	pthread_mutex_unlock(&_map_mutex);
}
