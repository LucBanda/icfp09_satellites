INCLUDE_FILES = $(shell find include/ -name \*.h)

BINARIES      = vm

OBJECTS = main.o vm_state.o instructions.o trace_generator.o controller.o meet_and_greed.o renderer.o satellite.o ellipse.o clear_sky.o excentric.o bin4.o bin3.o bin2.o bin1.o

ifeq ($(debug),1)
OPTIMIZE = -g
else
OPTIMIZE = -O3
endif

ifeq ($(generate),1)
GENERATE = -DGENERATE
endif

ifeq ($(allegro),1)
ALLEGRO = -DALLEGRO
endif


CPPFLAGS  = -MD -MP -Wall $(OPTIMIZE) $(ALLEGRO) $(GENERATE) -Wsign-compare -Wshadow -Woverloaded-virtual -DNOSSL -fPIC 



INCLUDE = 

DEPFILES += $(shell find -name \*.d)

all: $(BINARIES)

%.o : %.cpp %.h
	@echo "  [CPP]  $@"
	@g++ $(CPPFLAGS) $(INCLUDE) -c -o $@ $<

%.o : %.c %.h
	@echo "  [CC]   $@"
	@g++ $(CPPFLAGS) $(INCLUDE) -c -o $@ $<

$(BINARIES) : $(OBJECTS)
	@echo "[LD] $@"
	@g++ $(OBJECTS) -L/usr/lib `allegro-config --libs` -o $@

clean:
	rm -f $(BINARIES)
	rm -f $(OBJECTS) 
	rm -f *.d