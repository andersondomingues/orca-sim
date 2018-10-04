#this environment variable must point to the hemps path, where the hardware, software and tools folders are located

#Definition of Plasma toolchain
CFLAGS     = -O2 -Wall -c -s
GCC_MIPS   = mips-elf-gcc $(CFLAGS)
AS_MIPS    = mips-elf-as
LD_MIPS    = mips-elf-ld
DUMP_MIPS  = mips-elf-objdump
COPY_MIPS = mips-elf-objcopy -I elf32-bigmips -O binary

#Definition of MB-Lite toolchain
MB         = mb-gcc
AS         = mb-as
LD         = mb-ld
MB_OBJCOPY = mb-objcopy
MB_OBJDUMP = mb-objdump
XILFLAGS   =-mxl-soft-div -msoft-float -mxl-barrel-shift -mno-xl-soft-mul
CXXFLAGS   =-g -std=c99 -pedantic -Wall -O2 
LNKFLAGS	   =-Wl,-defsym -Wl,_STACK_SIZE=0x3000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000
LNKFLAGS2  =-Wl,-defsym -Wl,_STACK_SIZE=0x2000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000
MB_GCC     = $(MB) $(XILFLAGS) $(CXXFLAGS) $(LNKFLAGS2) $(LIBFLAGS) $(INCFLAGS) $(CCFLAGS)

#TOOLS
BIN2MEM       = bin2mem
CONVERT       = -gatomaker
RAM_GENERATOR = ram_generator

INCLUDE       = $(HEMPS_PATH)/software/include

#TASKS
TASK0_PATH = ../applications/disturbing/disturbingtaskB_0.c
TASK0_INCLUDE = ids_disturbing.h
TASK0_NAME = disturbingtaskB_0
TASK0_ID = 0
TASK0_TARGET = $(TASK0_NAME)_$(TASK0_ID)

TASK1_PATH = ../applications/disturbing/disturbingtaskA_0.c
TASK1_INCLUDE = ids_disturbing.h
TASK1_NAME = disturbingtaskA_0
TASK1_ID = 1
TASK1_TARGET = $(TASK1_NAME)_$(TASK1_ID)

#tasks boot code for Plasma processor
BOOT_TASK_SRC     = $(HEMPS_PATH)/software/include/bootTask.asm
BOOT_TASK         = bootTask
#kernel master source files
BOOT_MASTER_SRC   = $(HEMPS_PATH)/software/kernel/master/boot.S
BOOT_MASTER       = boot_master
KERNEL_MASTER_SRC = $(HEMPS_PATH)/software/kernel/master/kernel.c
KERNEL_MASTER     = kernel_master
#kernel slave plasma source files
BOOT_PLASMA_SRC   = $(HEMPS_PATH)/software/kernel/slave/boot.S
BOOT_PLASMA       = boot_plasma
KERNEL_PLASMA_SRC = $(HEMPS_PATH)/software/kernel/slave/kernel.c
KERNEL_PLASMA     = kernel_plasma
#kernel slave mblite source files
KERNEL_MBLITE     = kernel_mblite
INTERRUPT_SRC     = $(HEMPS_PATH)/software/kernel/slave/interrupt.s
INTERRUPT         = interrupt

bootTask:
	$(AS_MIPS) -o $(BOOT_TASK).o $(BOOT_TASK_SRC)

kernel_master:
	$(AS_MIPS) -o $(BOOT_MASTER).o $(BOOT_MASTER_SRC)
	$(GCC_MIPS) -o $(KERNEL_MASTER).o $(KERNEL_MASTER_SRC) --include ids_master.h --include InitializeVectorsMaster.h
	$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER).map -s -N -o $(KERNEL_MASTER).bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o
	$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER)_debug.map -o $(KERNEL_MASTER)_debug.bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o
	$(DUMP_MIPS) -S $(KERNEL_MASTER)_debug.bin > $(KERNEL_MASTER).lst
	$(COPY_MIPS) $(KERNEL_MASTER).bin $(KERNEL_MASTER).dump
	hexdump -v -e '1/1 "%02x" 1/1 "%02x" 1/1 "%02x" 1/1 "%02x" "\n"' $(KERNEL_MASTER).dump > $(KERNEL_MASTER).txt
	sed -i -e '4s/.*/37bdc800/' $(KERNEL_MASTER).txt

kernel_plasma:
	$(AS_MIPS) -o $(BOOT_PLASMA).o $(BOOT_PLASMA_SRC)
	$(GCC_MIPS) -o $(KERNEL_PLASMA).o $(KERNEL_PLASMA_SRC) --include ids_slave.h --include InitializeVectorsSlave.h -D PLASMA
	$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA).map -s -N -o $(KERNEL_PLASMA).bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o
	$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA)_debug.map -o $(KERNEL_PLASMA)_debug.bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o
	$(DUMP_MIPS) -S $(KERNEL_PLASMA)_debug.bin > $(KERNEL_PLASMA).lst
	$(COPY_MIPS) $(KERNEL_PLASMA).bin $(KERNEL_PLASMA).dump
	hexdump -v -e '1/1 "%02x" 1/1 "%02x" 1/1 "%02x" 1/1 "%02x" "\n"' $(KERNEL_PLASMA).dump > $(KERNEL_PLASMA).txt
	sed -i -e '4s/.*/37bd7800/' $(KERNEL_PLASMA).txt

kernel_mblite:
	@echo "generating dummy kernel for mblite processor"
	cp $(KERNEL_MASTER).txt $(KERNEL_MBLITE).txt

disturbingtaskB_0_0:
	$(GCC_MIPS) $(TASK0_PATH) -o $(TASK0_TARGET).o --include $(TASK0_INCLUDE) -D PLASMA -I $(INCLUDE)
	$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK0_TARGET).map -s -N -o $(TASK0_TARGET).bin $(BOOT_TASK).o $(TASK0_TARGET).o
	$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK0_TARGET)_debug.map -o $(TASK0_TARGET)_debug.bin $(BOOT_TASK).o $(TASK0_TARGET).o
	$(DUMP_MIPS) -S $(TASK0_TARGET)_debug.bin > $(TASK0_TARGET).lst
	$(COPY_MIPS) $(TASK0_TARGET).bin $(TASK0_TARGET).dump
	hexdump -v -e '1/1 "%02x" 1/1 "%02x" 1/1 "%02x" 1/1 "%02x" "\n"' $(TASK0_TARGET).dump > $(TASK0_TARGET).txt
	sed -i -e '4s/.*/37bd4000/' $(TASK0_TARGET).txt

disturbingtaskA_0_1:
	$(GCC_MIPS) $(TASK1_PATH) -o $(TASK1_TARGET).o --include $(TASK1_INCLUDE) -D PLASMA -I $(INCLUDE)
	$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK1_TARGET).map -s -N -o $(TASK1_TARGET).bin $(BOOT_TASK).o $(TASK1_TARGET).o
	$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK1_TARGET)_debug.map -o $(TASK1_TARGET)_debug.bin $(BOOT_TASK).o $(TASK1_TARGET).o
	$(DUMP_MIPS) -S $(TASK1_TARGET)_debug.bin > $(TASK1_TARGET).lst
	$(COPY_MIPS) $(TASK1_TARGET).bin $(TASK1_TARGET).dump
	hexdump -v -e '1/1 "%02x" 1/1 "%02x" 1/1 "%02x" 1/1 "%02x" "\n"' $(TASK1_TARGET).dump > $(TASK1_TARGET).txt
	sed -i -e '4s/.*/37bd4000/' $(TASK1_TARGET).txt

ram_gen: ram_master ram_plasma ram_mblite
	
ram_master:
	$(RAM_GENERATOR) -64 -vhd kernel_master.txt > ram_master.vhd
	cp ram_master.vhd ../plasma_ram/rtl
	$(RAM_GENERATOR) -64 -h kernel_master.txt > ram_master.h
	cp ram_master.h ../plasma_ram/sc

ram_plasma:
	$(RAM_GENERATOR) -64 -vhd kernel_plasma.txt > ram_plasma.vhd
	cp ram_plasma.vhd ../plasma_ram/rtl
	$(RAM_GENERATOR) -64 -h kernel_plasma.txt > ram_plasma.h
	cp ram_plasma.h ../plasma_ram/sc

ram_mblite:
	$(RAM_GENERATOR) -64 -vhd kernel_mblite.txt > ram_mblite.vhd
	cp ram_mblite.vhd ../mblite_ram/rtl
	$(RAM_GENERATOR) -64 -h kernel_mblite.txt > ram_mblite.h
	cp ram_mblite.h ../mblite_ram/sc

clean:
	rm -rf *.bin
	rm -rf *.txt
	rm -rf *.mem
	rm -rf *.dump
	rm -rf *.lst
	rm -rf *.o
	rm -rf *.map
	rm -rf ram*.h
	rm -rf *.vhd
	rm -rf *.elf

all: bootTask kernel_master kernel_plasma kernel_mblite disturbingtaskB_0_0 disturbingtaskA_0_1 ram_gen
