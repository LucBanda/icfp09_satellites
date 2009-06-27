INSTALL_PATH = $(shell /home/users/ref_designs/Software/platform)
#INSTALL_PATH = /tmp

INCLUDE_FILES = $(shell find include/ -name \*.h)

BINARIES      = vm


OBJECTS = main.o vm_state.o instructions.o trace_generator.o controller.o

OPTIMIZE= -g
#OPTIMIZE= -O3

CPPFLAGS  = -MD -MP -Wall $(OPTIMIZE) -Wsign-compare -Wshadow -Woverloaded-virtual -DNOSSL -fPIC

INCLUDE = 

DEPFILES += $(shell find -name \*.d)

all: $(BINARIES)

%.o : %.cpp %.h
	@echo "  [CPP]  $@"
	g++ $(CPPFLAGS) $(INCLUDE) -c -o $@ $<

%.o : %.c %.h
	@echo "  [CC]   $@"
	g++ $(CPPFLAGS) $(INCLUDE) -c -o $@ $<

$(BINARIES) : $(OBJECTS)
	@echo "[LD] $@"
	g++ $(OBJECTS) -o $@

clean:
	rm -f $(BINARIES)
	rm -f $(OBJECTS) 
	rm -f *.d