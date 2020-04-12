#directory configuration
SOURCEDIR :=./src
HEADERDIR :=./include
OUTPUTDIR :=./bin
LIBNAME :=librsv.a

LIBSIM_PATH:=../simulator
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

#include flags (plus external flags)
CPPFLAGS :=-I./include -I../simulator/include -c $(COMPLINE) $(GLOBAL_SETTINGS)

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/UdpAsyncServer.o \
	$(OUTPUTDIR)/RspServer.o

all: $(OUTPUTDIR)/$(LIBNAME)
	$(Q)g++ $(OUTPUTDIR)/*.o -o $(OUTPUTDIR)/a.out
	./$(OUTPUTDIR)/a.out

#compress all generated object files into a pack
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes (.cpp) into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp  $(HEADERDIR)/%.h 
	$(Q)g++ $(CPPFLAGS) $< -o $@ 

#remove any preoviosly generated file
clean:
	$(Q)rm -rf bin/*.a 	rm -rf bin/*.o

