#applications file 
SELECTED_APPS = producer-consumer example-echo-print

#platform-specific configuration
include ../Configuration.mk

#arch confs.
ARCH = riscv/hf-riscv
IMAGE_NAME = image

#dir config.
HFOS_DIR = hellfireos

CPU_ARCH = \"$(ARCH)\"
MAX_TASKS = 30
MUTEX_TYPE = 0
MEM_ALLOC = 3
HEAP_SIZE = 500000
FLOATING_POINT = 0
KERNEL_LOG = 0

SRC_DIR = $(HFOS_DIR)

#includes for kernel parts
include $(HFOS_DIR)/arch/$(ARCH)/arch.mak
include $(HFOS_DIR)/lib/lib.mak
include $(HFOS_DIR)/drivers/noc.mak
include $(HFOS_DIR)/sys/kernel.mak

#includes hardware-specific support
include orca-lib/orca-lib.mak

#include tasks 
include $(HFOS_DIR)/../applications/*/app.mak

#phonies
.PHONY: clean

INC_DIRS += -I $(HFOS_DIR)/lib/include -I $(HFOS_DIR)/sys/include -I $(HFOS_DIR)/drivers/noc/include -I $(HFOS_DIR)/../orca-lib/include

CFLAGS += -DCPU_ARCH=$(CPU_ARCH) \
	-DMAX_TASKS=$(MAX_TASKS) -DMEM_ALLOC=$(MEM_ALLOC) \
	-DHEAP_SIZE=$(HEAP_SIZE) -DMUTEX_TYPE=$(MUTEX_TYPE) \
	-DFLOATING_POINT=$(FLOATING_POINT) \
	-DKERNEL_LOG=$(KERNEL_LOG) \
	$(COMPLINE) \
	$(NOC_FLAGS) \
	-DDEBUG_PORT

NOC_FLAGS = -DNOC_INTERCONNECT -DNOC_PACKET_SIZE=64 -DNOC_PACKET_SLOTS=64 \
	    -DNOC_WIDTH=4 -DNOC_HEIGHT=4

image:
	make hal
	make libc
	make noc
	make kernel
	for app in $(SELECTED_APPS) ; do \
		make $$app.o ; \
	done
	make orca-lib.o
	$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) -o $(IMAGE_NAME).elf *.o
	$(DUMP) --disassemble --reloc $(IMAGE_NAME).elf > $(IMAGE_NAME).lst
	$(DUMP) -h $(IMAGE_NAME).elf > $(IMAGE_NAME).sec
	$(DUMP) -s $(IMAGE_NAME).elf > $(IMAGE_NAME).cnt
	$(OBJ) -O binary $(IMAGE_NAME).elf $(IMAGE_NAME).bin
	$(SIZE) $(IMAGE_NAME).elf
	hexdump -v -e '4/1 "%02x" "\n"' $(IMAGE_NAME).bin > $(IMAGE_NAME).txt
	mv *.o *.elf *.bin *.cnt *.lst *.sec *.txt ../bin
	
#image_apps:
#	for app in $(SELECTED_APPS) ; do \
#		make $$app.o ; \
#		$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) -o $$app.elf *.o -limage -L../bin ; \
#		$(OBJ) $$app.elf $$app.elf --only-section=.tasks ; \
#		$(DUMP) --disassemble --reloc $$app.elf > $$app.lst ; \
#		$(DUMP) -h $$app.elf > $$app.sec ; \
#		$(DUMP) -s $$app.elf > $$app.cnt ; \
#		$(OBJ) -O binary $$app.elf $$app.bin ; \
#		$(SIZE) $$app.elf ; \
#		hexdump -v -e '4/1 "%02x" "\n"' $$app.bin > $$app.txt ; \
#		mv $$app.o ../bin ; \
#	done
	
clean:
	rm -rf *.o *~ *.elf *.bin *.cnt *.lst *.sec *.txt

