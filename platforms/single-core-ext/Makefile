include Configuration.mk

# This is the name of the platform, which will be used 
# to name the generated executable file.
PLATFORM_NAME :=single-core-ext

# You can append other folders here if your project 
# depends on external libraries
OUTPUTDIR :=bin
SOURCEDIR :=src
HEADERDIR :=include

# ===================================================== LINKAGE
# Hardware modules to be linked to your project (the
# ones that will take part in the simulation)
LINKING := \
	-lhfriscv-core \
	-lorca-network-interface \
	-lorca-virtual-ethernet

# Orcasim modules that will be linked to your application.
LINKING += \
	-lorcasim-modeling \
	-lorcasim-gdbrsp \
	-lorcasim-base

# Other linking
LINKING += -lpthread

# ===================================================== INCLUDING
INCLUDES := \
	-I../../models/hfriscv-core/include \
	-I../../models/orca-network-interface/include \
	-I../../models/orca-virtual-ethernet/include

INCLUDES += \
	-I../../orcasim-gdbrsp/include \
	-I../../orcasim-modeling/include \
	-I../../orcasim-base/include \

INCLUDES += -Iinclude

# ===================================================== FLAGS
#CXX_FLAGS += -O3 -march=native -mtune=native -fmax-errors=5 -std=c++17 \
#	$(CONFIG_MEMORY) $(INCLUDES) -flto

CXX_FLAGS += $(CONFIG_MEMORY) $(INCLUDES) -ggdb

LD_FLAGS += -Lbin $(LINKING)

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.hpp) $(wildcard $(HEADERDIR)/*.h)
SOURCE_FILES := $(wildcard $(SOURCEDIR)/*.cpp)

# ===================================================== 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCE_FILES) $(HEADER_FILES)
	#compile libraries
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

