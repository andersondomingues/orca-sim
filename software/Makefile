ARCH_GCC :=riscv32-unknown-elf-gcc
ARCH_LD  :=riscv32-unknown-elf-ld
ARCH_OBJ :=riscv32-unknown-elf-objcopy

GCCFLAGS := -march=rv32i -mabi=ilp32 -Wall -O2 -c -ffreestanding -nostdlib -ffixed-s10 -ffixed-s11 -DLITTLE_ENDIAN -fdata-sections -ffunction-sections

LDFLAGS := -melf32lriscv --gc-sections

OUTPUTDIR :=bin


all: $(OUTPUTDIR)/sort.bin
	@echo "done."

libc.o: libc.c 
	$(ARCH_GCC) $(GCCFLAGS) libc.c -o libc.o -DCPU_SPEED=1

crt0.o: crt0.s
	$(ARCH_GCC) $(GCCFLAGS) crt0.s -o crt0.o

sort.o: sort/sort.c
	$(ARCH_GCC) $(GCCFLAGS) sort/sort.c -o sort.o
	
sort.elf: sort.o crt0.o libc.o
	$(ARCH_LD) $(LDFLAGS) risc-v.ld -Map sort.map -N -o sort.elf sort.o crt0.o libc.o

$(OUTPUTDIR)/sort.bin: sort.elf
	$(ARCH_OBJ) -O binary sort.elf sort.bin --pad-to 4096
	mv sort.bin $(OUTPUTDIR)/sort.bin

clean:
	rm -rf *.o *.elf *.map *.bin