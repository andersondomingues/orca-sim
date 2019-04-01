#arch confs.
ARCH = riscv/hf-riscv
IMAGE_NAME = image

#applications file 
SELECTED_APPS = noc_test4_sender noc_test4_receiver

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

#includes for loader and other software
include orca-launcher/orca-launcher.mak

#include for applications
include $(HFOS_DIR)/../applications/noc_test4/app.mak

#phonies
.PHONY: clean image_apps

INC_DIRS += -I $(HFOS_DIR)/lib/include -I $(HFOS_DIR)/sys/include -I $(HFOS_DIR)/drivers/noc/include
CFLAGS += -DCPU_ARCH=$(CPU_ARCH) \
	-DMAX_TASKS=$(MAX_TASKS) -DMEM_ALLOC=$(MEM_ALLOC) \
	-DHEAP_SIZE=$(HEAP_SIZE) -DMUTEX_TYPE=$(MUTEX_TYPE) \
	-DFLOATING_POINT=$(FLOATING_POINT) -DKERNEL_LOG=$(KERNEL_LOG) \
	$(NOC_FLAGS) -DDEBUG_PORT

NOC_FLAGS = -DNOC_INTERCONNECT -DNOC_WIDTH=4 -DNOC_HEIGHT=4 -DNOC_PACKET_SIZE=64 -DNOC_PACKET_SLOTS=64

image:
	make hal
	make libc
	make noc
	make kernel
	make orca-launcher.o
	$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) -o $(IMAGE_NAME).elf *.o
	$(DUMP) --disassemble --reloc $(IMAGE_NAME).elf > $(IMAGE_NAME).lst
	$(DUMP) -h $(IMAGE_NAME).elf > $(IMAGE_NAME).sec
	$(DUMP) -s $(IMAGE_NAME).elf > $(IMAGE_NAME).cnt
	$(OBJ) -O binary $(IMAGE_NAME).elf $(IMAGE_NAME).bin
	$(SIZE) $(IMAGE_NAME).elf
	hexdump -v -e '4/1 "%02x" "\n"' $(IMAGE_NAME).bin > $(IMAGE_NAME).txt
	make image_apps
	mv *.o *.elf *.bin *.cnt *.lst *.sec *.txt ../bin

image_apps:
	for app in $(SELECTED_APPS) ; do \
		make $$app.o ; \
		$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) -o $$app.elf *.o ; \
		$(OBJ) $$app.elf $$app.elf --only-section=.tasks ; \
		$(DUMP) --disassemble --reloc $$app.elf > $$app.lst ; \
		$(DUMP) -h $$app.elf > $$app.sec ; \
		$(DUMP) -s $$app.elf > $$app.cnt ; \
		$(OBJ) -O binary $$app.elf $$app.bin ; \
		$(SIZE) $$app.elf ; \
		hexdump -v -e '4/1 "%02x" "\n"' $$app.bin > $$app.txt ; \
		mv $$app.o ../bin ; \
	done
	
clean:
	rm -rf *.o *~ *.elf *.bin *.cnt *.lst *.sec *.txt

