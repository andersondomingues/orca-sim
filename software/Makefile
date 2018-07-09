TOOLCHAIN_DIR :=riscv32-unknown-elf/gcc-7.1.0/bin/

ARCH_GCC :=$(TOOLCHAIN_DIR)/riscv32-unknown-elf-gcc
ARCH_LD  :=$(TOOLCHAIN_DIR)/riscv32-unknown-elf-ld
ARCH_OBJ :=$(TOOLCHAIN_DIR)/riscv32-unknown-elf-objcopy

GCCFLAGS := -march=rv32i -mabi=ilp32 -Wall -O2 -c -ffreestanding -nostdlib -ffixed-s10 -ffixed-s11 -DLITTLE_ENDIAN -fdata-sections -ffunction-sections

LDFLAGS :=

libc.o: 
	$(ARCH_ELF) $(GCCFLAGS) libc.c -o libc.o -DCPU_SPEED=1

crt0.o:
	$(ARCH_ELF) $(GCCFLAGS) crt0.s -o crt0.o

sort.o:
	$(ARCH_ELF) $(GCCFLAGS) sort.c -o sort.o
	
sort.elf:
	$(ARCH_ELF) -melf32lriscv --gc-sections risc-v.ld -Map sort.map -N -o sort.elf sort.o crt0.o libc.o

sort.bin:
	$(ARCH_OBJ) ARCH_OBJ -O binary sort.elf sort.bin --pad-to 4096
	mv sort.bin ../output/sort.bin
	rm -rf *.o *.elf *.map