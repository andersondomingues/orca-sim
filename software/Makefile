#apps and arch confs.
APP = $(APPLICATION)
ARCH = riscv/hf-riscv

#dir config.
HFOS_DIR = ./hellfireos
APPS_DIR = ./applications

CPU_ARCH = \"$(ARCH)\"
MAX_TASKS = 30
MUTEX_TYPE = 0
MEM_ALLOC = 3
HEAP_SIZE = 500000
FLOATING_POINT = 0
KERNEL_LOG = 0

SRC_DIR = $(HFOS_DIR)

include $(HFOS_DIR)/arch/$(ARCH)/arch.mak
include $(HFOS_DIR)/lib/lib.mak
include $(HFOS_DIR)/drivers/noc.mak
include $(HFOS_DIR)/sys/kernel.mak
include $(APPS_DIR)/$(APP)/app.mak

include orca-launcher/orca-launcher.mak

INC_DIRS += -I $(HFOS_DIR)/lib/include -I $(HFOS_DIR)/sys/include -I $(HFOS_DIR)/drivers/noc/include
CFLAGS += -DCPU_ARCH=$(CPU_ARCH) \
	-DMAX_TASKS=$(MAX_TASKS) -DMEM_ALLOC=$(MEM_ALLOC) \
	-DHEAP_SIZE=$(HEAP_SIZE) -DMUTEX_TYPE=$(MUTEX_TYPE) \
	-DFLOATING_POINT=$(FLOATING_POINT) -DKERNEL_LOG=$(KERNEL_LOG) \
	$(NOC_FLAGS) -DDEBUG_PORT

NOC_FLAGS = -DNOC_INTERCONNECT -DNOC_WIDTH=4 -DNOC_HEIGHT=4 -DNOC_PACKET_SIZE=64 -DNOC_PACKET_SLOTS=64

images: 
	make hal
	make libc
	make noc
	make kernel
	make orca-launcher
	$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) -o code0.elf *.o
	$(DUMP) --disassemble --reloc code0.elf > code0.lst
	$(DUMP) -h code0.elf > code0.sec
	$(DUMP) -s code0.elf > code0.cnt
	$(OBJ) -O binary code0.elf code0.bin
	$(SIZE) code0.elf
	hexdump -v -e '4/1 "%02x" "\n"' code0.bin > code0.txt
	mv *.o *.elf *.bin *.cnt *.lst *.sec *.txt ../bin

app: 
	make app

clean:
	rm -rf ../bin/*.o ../bin/*~ ../bin/*.elf ../bin/*.bin \
	../bin/*.cnt ../bin/*.lst ../bin/*.sec ../bin/*.txt

