# include local config
include Configuration.mk
SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

# Name of the package
LIBNAME := liborca-modeling.a 

# Dependencies
DEPS_INCLUDE := -I"../orcasim-base/include"
DEPS_LINKAGE := -lorcasim-base

# Add optimization and include flags to the compilation. Compilation 
# optimizations favor performance over code size.
CXXFLAGS := \
	-O3 -march=native -mtune=native \
	-I$(HEADERDIR) $(DEPS_INCLUDE)

TARGET_DEPS := \
	$(OUTPUTDIR)/ProcessorBase.o \
	$(OUTPUTDIR)/Signal.o \
	$(OUTPUTDIR)/SignalSet.o \
	$(OUTPUTDIR)/Memory.o \
	$(OUTPUTDIR)/Buffer.o

# Pack object code into a static library
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

# Compile classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.hpp 
	$(Q)g++ $(CXXFLAGS) $< -o $@ -c
	
# Remove generated object files and library
clean:
	$(Q)rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a