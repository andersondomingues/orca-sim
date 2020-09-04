#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME :=SingleCoreExt

OUTPUTDIR :=./bin
SOURCEDIR :=./src
HEADERDIR :=./include

# required libraries
DEP_LIBS := \
	orcasim-base \
	orcasim-modeling \
	orcasim-gdbrsp

DEP_LINKING := \
	-lorcasim-base \
	-lorcasim-modeling \
	-lorcasim-gdbrsp \
	-lpthread

DEP_INCLUDES := \
	-I../../orcasim-base \
	-I../../orcasim-modeling \
	-I../../orcasim-gdbrsp

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
INCLUDES = -Iinclude $(DEP_INCLUDES)
CXX_FLAGS += $(INCLUDES) -O3 -march=native -mtune=native

LD_FLAGS += -Lbin $(DEP_LINKING)


# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.hpp)
SOURCE_FILES := $(wildcard $(SOURCEDIR)/*.cpp)

#source files for the platform
TARGET_DEPS := $(SOURCEDIR)/Orca.cpp

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCE_FILES) $(HEADER_FILES)
	make -C ../../orcasim-base
	make -C ../../orcasim-modeling
	make -C ../../orcasim-gdbrsp
	$(Q)g++ $(SOURCE_FILES) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(LD_FLAGS) $(CXXFLAGS)

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe

