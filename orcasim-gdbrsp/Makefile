SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

# Name of the package
LIBNAME := liborca-gdbrsp.a 

# Dependencies
ORCASIM_BASE_DIR =../orcasim-base
ORCASIM_MODELING_DIR =../orcasim-modeling
DEPS_INCLUDE = -I$(ORCASIM_BASE_DIR)/include -I$(ORCASIM_MODELING_DIR)/include

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/UdpAsyncServer.o \
	$(OUTPUTDIR)/RspServer.o \
	$(OUTPUTDIR)/DataConvertionHelper.o \
	$(OUTPUTDIR)/GdbProcessorBase.o 

# Add optimization and include flags to the compilation. Compilation 
# optimizations favor performance over code size.
CXXFLAGS := -fmax-errors=10 \
	-O3 -march=native -mtune=native \
	-I$(HEADERDIR) $(DEPS_INCLUDE)

#pack file into a static library to be used later
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.hpp 
	$(Q)g++ $(CXXFLAGS) $< -o $@ -c

# Remove generated object files and library
clean:
	$(Q)rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a



