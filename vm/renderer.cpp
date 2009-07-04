#include "common.h"
#include <complex>

#include "renderer.h"
#include "vm_state.h"
#include "ellipse.h"


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
	  
	  circlefill(bmp, 500+real((*it)->position())/(SCALE),  500-imag((*it)->position())/(SCALE) , 4, color);
	  
	  
	  
	  if ((*it)->trajectoire() == NULL) {
		if ((*it)->main_sat())
		  circle(bmp, 500,500, start_radius/SCALE, CRATER_COL);
		else
		  circle(bmp, 500,500, abs((*it)->orbit())/SCALE, CRATER_COL);
		if ((*it)->orbit() > SCALE*500)
		  SCALE = (*it)->orbit()/450;
	  } else{
		for (vector<complex<double> >::iterator traj_iter = (*it)->trajectoire()->_trace.begin(); traj_iter < (*it)->trajectoire()->_trace.end(); traj_iter+=((*it)->trajectoire()->_trace.size()+5000) / 5000) {
		  putpixel(bmp, 500+real(*traj_iter)/(SCALE),  500-imag(*traj_iter)/(SCALE) , CRATER_COL);
		  if (abs(*traj_iter) > SCALE*500)
			SCALE = abs(*traj_iter)/450;
		}
		if ((*it)->trajectoire()->is_defined()) {
			line(bmp, 500+real((*it)->trajectoire()->_apogee)/SCALE,500-imag((*it)->trajectoire()->_apogee)/SCALE,  500+real((*it)->trajectoire()->_perige)/SCALE, 500-imag((*it)->trajectoire()->_perige)/SCALE, CRATER_COL);
		  }
	  }
	  
	  
	}
	
	for (vector<double>::iterator it = _radius.begin(); it != _radius.end(); it++) {
	  if (*it > SCALE * 500)
		SCALE = *it/450;
	  circle(bmp, 500,500,*it/SCALE, CRATER_COL);
	}
	  
	unlock();
#endif
}

void renderer::add_radius(double radius) {
  _radius.push_back(radius);
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
	pthread_mutex_init(&_main_mutex, NULL);
	pthread_create(&_mainthread, NULL, renderer::mainLoop, NULL);
#endif
}
				 // Fonctions de création et destruction du singleton
void renderer::terminate()
{
#ifdef ALLEGRO
	_running =false;
	pthread_mutex_destroy(&_main_mutex);
	pthread_join(_mainthread, NULL);
#endif
}


void renderer::lock()
{
  pthread_mutex_lock( &_main_mutex );
}
void renderer::unlock()
{
  pthread_mutex_unlock( &_main_mutex );
}
