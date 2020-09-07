SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

# Name of the package
LIBNAME := liborcasim-base.a 

TARGET_DEPS := \
	$(OUTPUTDIR)/Event.o \
	$(OUTPUTDIR)/Model.o \
	$(OUTPUTDIR)/TimedModel.o \
	$(OUTPUTDIR)/UntimedModel.o \
	$(OUTPUTDIR)/Engine.o 

# Add optimization and include flags to the compilation. Compilation 
# optimizations favor performance over code size.
CXXFLAGS := -O3 -march=native -mtune=native -std=c++17 -fmax-errors=5

#pack file into a static library to be used later
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	@ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)
	@ar -t $(OUTPUTDIR)/$(LIBNAME)


#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp $(HEADERDIR)/%.hpp 
	@g++ $(CXXFLAGS) $< -o $@ -c -I$(HEADERDIR)

# Remove generated object files and library
clean:
	@rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a



