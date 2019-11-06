#dir config
SOURCEDIR :=./src
HEADERDIR :=./include
OUTPUTDIR :=./bin
LIBNAME := libsim.a 

CPPFLAGS := -I$(HEADERDIR) -c $(COMPLINE) $(GLOBAL_SETTINGS)

TARGET_DEPS := \
	$(OUTPUTDIR)/Event.o \
	$(OUTPUTDIR)/Model.o \
	$(OUTPUTDIR)/TimedModel.o \
	$(OUTPUTDIR)/UntimedModel.o \
	$(OUTPUTDIR)/Simulator.o 

#pack file into a static library to be used later
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ 
	
#remove previously generated files
clean:
	rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a
