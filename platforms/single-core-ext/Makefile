include Configuration.mk
#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME :=SingleCoreExt

OUTPUTDIR :=bin
SOURCEDIR :=src
HEADERDIR :=include

#-Wl,--start-group 
LINKING := \
	-lhfriscv-core \
	-lorca-network-interface \
	-lorca-virtual-ethernet \
	-lorcasim-modeling \
	-lorcasim-gdbrsp \
	-lorcasim-base \
	-lpthread 

INCLUDES := \
	-I../../models/hfriscv-core/include \
	-I../../models/orca-network-interface/include \
	-I../../models/orca-virtual-ethernet/include \
	-I../../orcasim-gdbrsp/include \
	-I../../orcasim-modeling/include \
	-I../../orcasim-base/include \
	-Iinclude

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
CXX_FLAGS += -O3 -march=native -mtune=native -fmax-errors=5 -std=c++17 \
	$(CONFIG_MEMORY) $(INCLUDES) 
LD_FLAGS += -Lbin $(LINKING)

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.hpp) $(wildcard $(HEADERDIR)/*.h)
SOURCE_FILES := $(wildcard $(SOURCEDIR)/*.cpp)

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCE_FILES) $(HEADER_FILES)
	make -C ../../orcasim-base
	make -C ../../orcasim-modeling
	make -C ../../orcasim-gdbrsp
	cp ../../orcasim-base/bin/*.a ./bin
	cp ../../orcasim-modeling/bin/*.a ./bin
	cp ../../orcasim-gdbrsp/bin/*.a ./bin
	make -C ../../models/orca-virtual-ethernet
	make -C ../../models/hfriscv-core
	make -C ../../models/orca-network-interface
	cp ../../models/orca-virtual-ethernet/bin/*.a ./bin
	cp ../../models/hfriscv-core/bin/*.a ./bin
	cp ../../models/orca-network-interface/bin/*.a ./bin
	g++ $(CXX_FLAGS) -Lbin -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(SOURCE_FILES) \
		-Wl,--start-group $(LINKING)
#ld --start-group $(OUTPUTDIR)/$(PLATFORM_NAME).o bin/*.a --end-group \
#		-o $(OUTPUTDIR)/$(PLATFORM_NAME).exe

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/*.exe $(OUTPUTDIR)/*.a $(OUTPUTDIR)/*.o 
	make -C ../../orcasim-base clean
	make -C ../../orcasim-modeling clean
	make -C ../../orcasim-gdbrsp clean
	make -C ../../models/orca-virtual-ethernet clean
	make -C ../../models/hfriscv-core clean
	make -C ../../models/orca-network-interface clean

