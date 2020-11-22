# include local config
include Configuration.mk
SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

# Name of the package
LIBNAME := liborcasim-modeling.a 

# Dependencies
DEPS_INCLUDE := -I"../orcasim-base/include"
DEPS_LINKAGE := -lorcasim-base

# Add optimization and include flags to the compilation. Compilation 
# optimizations favor performance over code size.
#CXXFLAGS := \
#	-O3 -march=native -mtune=native -fmax-errors=5 -std=c++17 \

CXXFLAGS := -fmax-errors=5 -std=c++17 -I$(HEADERDIR) $(DEPS_INCLUDE) -ggdb


TARGET_DEPS := \
	$(OUTPUTDIR)/ProcessorBase.o \
	$(OUTPUTDIR)/Signal.o \
	$(OUTPUTDIR)/SignalSet.o \
	$(OUTPUTDIR)/Memory.o \
	$(OUTPUTDIR)/Buffer.o \
	$(OUTPUTDIR)/Simulator.o

# Pack object code into a static library
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	@ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)
	@ar -t $(OUTPUTDIR)/$(LIBNAME)

# Compile classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.hpp 
	@g++ $(CXXFLAGS) $< -o $@ -c 
	
# Remove generated object files and library
clean:
	rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a