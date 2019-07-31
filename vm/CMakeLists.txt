cmake_minimum_required(VERSION 2.6)
PROJECT (icfp09)

set(CONFIG_DEBUG OFF CACHE BOOL "")

if(CONFIG_DEBUG)
	add_definitions(-g -std=c++11)
else(CONFIG_DEBUG)
	add_definitions(-O3 -std=c++11)
endif(CONFIG_DEBUG)


if(CONFIG_ALLEGRO)
	add_definitions(-DALLEGRO)
endif(CONFIG_ALLEGRO)


if(CONFIG_GENERATE)
	add_definitions(-DGENERATE)
endif(CONFIG_GENERATE)

include_directories (. /usr/local/include ../openGA/src)
link_directories(/usr/lib /usr/bin /usr/local/lib)

add_definitions(-MD -MP -Wall -Wsign-compare -g -O0 -Wshadow -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (vm main.cpp vm_state.cpp renderer.cpp bin4.cpp bin3.cpp bin2.cpp bin1.cpp agent.cpp)
add_executable (optimizer genetic_optimizer.cpp vm_state.cpp bin4.cpp bin3.cpp bin2.cpp bin1.cpp agent.cpp)

target_link_libraries(vm allegro allegro_primitives allegro_main)
target_link_libraries(optimizer allegro allegro_main)