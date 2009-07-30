cmake_minimum_required(VERSION 2.6)
PROJECT (icfp09)

set(DEBUG OFF CACHE BOOL "")
set(ALLEGRO ON CACHE BOOL "")
set(GENERATE OFF CACHE BOOL "")

if(DEBUG)
	add_definitions(-g)
else(DEBUG)
	add_definitions(-O3)
endif(DEBUG)


if(ALLEGRO)
	add_definitions(-DALLEGRO)
endif(ALLEGRO)


if(GENERATE)
	add_definitions(-DGENERATE)
endif(GENERATE)

include_directories (.)
link_directories(/usr/lib /usr/bin)


add_definitions( -MD -MP -Wall -Wsign-compare -Wshadow -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (vm main.cpp vm_state.cpp instructions.cpp trace_generator.cpp controller.cpp meet_and_greed.cpp renderer.cpp satellite.cpp ellipse.cpp clear_sky.cpp excentric.cpp bin4.cpp bin3.cpp bin2.cpp bin1.cpp)

target_link_libraries(vm alleg-4.2.2)
