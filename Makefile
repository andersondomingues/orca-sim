#folder config
SOURCEDIR :=./src
HEADERDIR :=./include
OUTPUTDIR :=./build
LIBNAME := libursa.a

#include configuration file 
include Configuration.mk

#add parameter to the compiling 
ifeq ($(URSA_ZERO_TIME_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(URSA_QUEUE_SIZE_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

#buffer parameters
ifeq ($(BUFFER_OVERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(BUFFER_UNDERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif
export COMPLINE

CPPFLAGS := -I$(HEADERDIR) -c $(COMPLINE) $(GLOBAL_SETTINGS)

#manually added all source dependencies
TARGET_DEPS := \
	$(OUTPUTDIR)/Event.o \
	$(OUTPUTDIR)/Model.o \
	$(OUTPUTDIR)/TimedModel.o \
	$(OUTPUTDIR)/UntimedModel.o \
	$(OUTPUTDIR)/Simulator.o 

#pack file into a static library to be used later
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.h
	g++ $(CPPFLAGS) $< -o $@ 
	
#remove previously generated files
clean:
	rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a
	
#documentation
docs:
	doxygen
