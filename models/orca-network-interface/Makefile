SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

# Name of the package
LIBNAME := liborca-network-interface.a 

# Dependencies
DEPS_INCLUDE := \
	-I"../../orcasim-base/include" \
	-I"../../orcasim-modeling/include" \
	-I"../../orcasim-gdbrsp/include" 

DEPS_LINKAGE := -lorcasim-base -lorcasim-modeling -lorcasim-gdbrsp

# Add optimization and include flags to the compilation. Compilation 
# optimizations favor performance over code size.
CXXFLAGS := \
	-fmax-errors=5 -std=c++17 -ggdb \
	-I$(HEADERDIR) $(DEPS_INCLUDE)
#-O3 -march=native -mtune=native -fmax-errors=5 -std=c++17 \

TARGET_DEPS := $(OUTPUTDIR)/DmaNetif.o

# Pack object code into a static library
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

# Compile classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.hpp 
	$(Q)g++ $(CXXFLAGS) $< -o $@ -c

# Remove generated object files and library
clean:
	$(Q)rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a