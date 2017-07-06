cmake_minimum_required(VERSION 2.6)
PROJECT (icfp09)

set(CONFIG_DEBUG OFF CACHE BOOL "")
set(CONFIG_ALLEGRO ON CACHE BOOL "")
set(CONFIG_GENERATE OFF CACHE BOOL "")
set(CONFIG_START_SLOW 10 CACHE INT "")

if(CONFIG_DEBUG)
	add_definitions(-g)
else(CONFIG_DEBUG)
	add_definitions(-O3)
endif(CONFIG_DEBUG)


if(CONFIG_ALLEGRO)
	add_definitions(-DALLEGRO)
endif(CONFIG_ALLEGRO)


if(CONFIG_GENERATE)
	add_definitions(-DGENERATE)
endif(CONFIG_GENERATE)

include_directories (.)
link_directories(/usr/lib /usr/bin)


add_definitions( -DSTART_TIMER=${CONFIG_START_SLOW} -MD -MP -Wall -Wsign-compare -Wshadow -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (vm main.cpp vm_state.cpp instructions.cpp trace_generator.cpp controller.cpp meet_and_greed.cpp renderer.cpp satellite.cpp ellipse.cpp clear_sky.cpp excentric.cpp bin4.cpp bin3.cpp bin2.cpp bin1.cpp)

target_link_libraries(vm alleg pthread)
