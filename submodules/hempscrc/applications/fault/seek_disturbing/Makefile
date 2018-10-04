LIB=work

#this environment variable must point to the hemps path, where the hardware, software and tools folders are located
BASE_PATH=$(HEMPS_PATH)
HW_PATH=hardware

#VHDL files
PKG_SRC=HeMPS_defaults.vhd
PKG_DIR=$(HW_PATH)
PKG_PATH=$(addprefix $(PKG_DIR)/,$(PKG_SRC))

SCENARIO_SRC=HeMPS_PKG repository dynamic_apps
SCENARIO_DIR=./
SCENARIO_PATH=$(addprefix $(SCENARIO_DIR)/,$(SCENARIO_SRC))

MPACK_SRC=mlite_pack.vhd UartFile.vhd
MPACK_DIR=$(HW_PATH)/plasma/rtl
MPACK_PATH=$(addprefix $(MPACK_DIR)/,$(MPACK_SRC))

MLITE_SRC=alu.vhd bus_mux.vhd control.vhd mem_ctrl.vhd mult.vhd pc_next.vhd pipeline.vhd reg_bank.vhd shifter.vhd mlite_cpu.vhd
MLITE_DIR=$(HW_PATH)/plasma/rtl
MLITE_PATH=$(addprefix $(MLITE_DIR)/,$(MLITE_SRC))

#NI_SRC=network_interface.vhd
NI_SRC=Network_Interface.vhd
NI_DIR=$(HW_PATH)/ni/rtl
NI_PATH=$(NI_DIR)/$(NI_SRC)

DMA_SRC=dma.vhd dma_master.vhd
DMA_DIR=$(HW_PATH)/dma/rtl
DMA_PATH=$(addprefix $(DMA_DIR)/,$(DMA_SRC))

#ROUTER_SRC=Hermes_buffer.vhd Hermes_crossbar.vhd Hermes_switchcontrol.vhd RouterCC.vhd
ROUTER_SRC=seek/stepdet.vhd seek/seek.vhd Input_Buffer.vhd CS_Buffer.vhd Crossbars.vhd Switch_Control.vhd mux_control.vhd Router_seek.vhd
ROUTER_DIR=$(HW_PATH)/router/rtl
ROUTER_PATH=$(addprefix $(ROUTER_DIR)/,$(ROUTER_SRC))

PLASMA_RAM_SRC=ram_master ram_plasma
PLASMA_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/rtl
PLASMA_RAM_PATH=$(addprefix $(PLASMA_RAM_DIR)/,$(PLASMA_RAM_SRC))

MBLITE_RAM_SRC=ram_mblite
MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/rtl
MBLITE_RAM_PATH=$(MBLITE_RAM_DIR)/$(MBLITE_RAM_SRC)

MBLITE_SRC=mblite_soc.vhd
MBLITE_DIR=$(HW_PATH)/mblite/rtl
MBLITE_PATH=$(addprefix $(MBLITE_DIR)/,$(MBLITE_SRC))

MBLITE_CORE_SRC=../config_Pkg.vhd core_Pkg.vhd core.vhd fetch.vhd gprf.vhd decode.vhd execute.vhd mem.vhd
MBLITE_CORE_DIR=$(MBLITE_DIR)/core
MBLITE_CORE_PATH=$(addprefix $(MBLITE_CORE_DIR)/,$(MBLITE_CORE_SRC))

WRAPPER_CELL_SRC=wrapper_cell.vhd wrapper_cell_N.vhd
WRAPPER_CELL_DIR=$(HW_PATH)
WRAPPER_CELL_PATH=$(addprefix $(WRAPPER_CELL_DIR)/,$(WRAPPER_CELL_SRC))

MBLITE_STD_SRC=std_Pkg.vhd dsram.vhd
MBLITE_STD_DIR=$(MBLITE_DIR)/std
MBLITE_STD_PATH=$(addprefix $(MBLITE_STD_DIR)/,$(MBLITE_STD_SRC))

PLASMA_SRC=access_repository.vhd plasma.vhd
PLASMA_DIR=$(HW_PATH)/plasma/rtl
PLASMA_PATH=$(addprefix $(PLASMA_DIR)/,$(PLASMA_SRC))

TOP_SRC=processing_element.vhd HeMPS.vhd insert_application.vhd test_bench.vhd
TOP_DIR=$(HW_PATH)
TOP_PATH=$(addprefix $(TOP_DIR)/,$(TOP_SRC))

TOP_WRAPPED_SRC=processing_element_wrapped.vhd HeMPS.vhd insert_application.vhd test_bench.vhd
TOP_WRAPPED_DIR=$(HW_PATH)
TOP_WRAPPED_PATH=$(addprefix $(TOP_WRAPPED_DIR)/,$(TOP_WRAPPED_SRC))

#SystemC files
SC_NOC_DIR=$(HW_PATH)/router/sc
SC_MLITE_DIR=$(HW_PATH)/plasma/sc
SC_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/sc
SC_MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/sc
SC_PLASMA_DIR=$(HW_PATH)/plasma/sc
SC_TESTBENCH_DIR=$(HW_PATH)/sc
SC_NI_DIR=$(HW_PATH)/ni/sc
SC_DMA_DIR=$(HW_PATH)/dma/sc
SC_ACCESS_REPO_DIR=$(HW_PATH)/plasma/sc

SC_NOC_SRC=queue.cpp switchcontrol.cpp router_cc.cpp 
SC_MLITE_SRC=mlite_cpu.cpp
SC_RAM_SRC=ram_master ram_plasma
SC_MBLITE_RAM_SRC=ram_mblite
SC_PLASMA_SRC=plasma_master.cpp plasma_slave.cpp
SC_TESTBENCH_SRC=hemps.cpp test_bench.cpp
SC_NI_SRC=Network_Interface.cpp
SC_DMA_SRC=dma.cpp dma_master.cpp
SC_ACCESS_REPO_SRC=access_repository.cpp

SC_NOC_PATH=$(addprefix $(SC_NOC_DIR)/,$(SC_NOC_SRC))
SC_MLITE_PATH=$(addprefix $(SC_MLITE_DIR)/,$(SC_MLITE_SRC))
SC_RAM_PATH=$(addprefix $(SC_RAM_DIR)/,$(SC_RAM_SRC))
SC_MBLITE_RAM_PATH=$(SC_MBLITE_RAM_DIR)/$(SC_MBLITE_RAM_SRC)
SC_PLASMA_PATH=$(addprefix $(SC_PLASMA_DIR)/,$(SC_PLASMA_SRC))
SC_TESTBENCH_PATH=$(addprefix $(SC_TESTBENCH_DIR)/,$(SC_TESTBENCH_SRC))
SC_NI_PATH=$(addprefix $(SC_NI_DIR)/,$(SC_NI_SRC))
SC_DMA_PATH=$(addprefix $(SC_DMA_DIR)/,$(SC_DMA_SRC))
SC_ACCESS_REPO_PATH=$(addprefix $(SC_ACCESS_REPO_DIR)/,$(SC_ACCESS_REPO_SRC))

#compilers definitions
INC=-Iplasma_ram/sc/ -I./
VHD_C=@vcom
GEN=g++ -o HeMPS.exe *.o -L. -Linc/. -lsystemc

#modelsim gcc compiler
SC_C=@sccom -work $(LIB) -g
#systemc g++ compiler
#SC_C=g++ -c -g -Wall -O2


default:
	@echo "Makefile for the hemps in systemc"
	@echo "---------------------------------------"
	@echo "Make options:"
	@echo "   default: Echo these instructions"
	@echo "       lib: Generate work dir and map its library"
	@echo "       vhd: Compile vhd HeMPS description files"
	@echo "       iss: Compile vhd and SystemC HeMPS description files"
	@echo "       all: Compile sytemc and vhd files"
	@echo "     clean: Remove all compiled and generated files"
	@echo

iss: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(SC_RAM_PATH) $(PLASMA_PATH) $(TOP_PATH)
#iss: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(SC_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(SC_MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)
	@sccom -link
	
iss_wrapped: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(SC_RAM_PATH) $(PLASMA_PATH) $(WRAPPER_CELL_PATH) $(TOP_WRAPPED_PATH)
	@sccom -link

scmod: lib $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)
	@sccom -link
	

sc: $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)
	$(GEN)

vhd: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(PLASMA_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)
	
$(SCENARIO_PATH):
	$(VHD_C) -work $(LIB) $(@).vhd
	
$(PLASMA_RAM_PATH):
	$(VHD_C) -work $(LIB) $(@).vhd
	
$(MBLITE_RAM_PATH):
	$(VHD_C) -work $(LIB) $(@).vhd
	
$(SC_RAM_PATH):
	$(SC_C) $(@).cpp  $(INC)
	
$(SC_MBLITE_RAM_PATH):
	$(SC_C) $(@).cpp  $(INC)

%.vhd:
	$(VHD_C) -work $(LIB) $(BASE_PATH)/$*.vhd
	
%.cpp:
	$(SC_C) $(BASE_PATH)/$*.cpp  $(INC)
	
sim:
	do sim.do
	
lib:
	@vlib $(LIB)
	@vmap $(LIB) $(LIB)
	
clean:
	@rm -r -f $(LIB)
	@rm -f transcript
	@rm -f modelsim.ini
	@rm -f vsim.wlf
	@rm -f *~
	@rm -f *.o
	@rm -f *.exe
	
all: iss_wrapped
	
