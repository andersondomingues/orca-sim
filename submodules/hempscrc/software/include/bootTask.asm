##################################################################
# TITLE: Tasks Boot Up Code 
# AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
#         Ismael Augusto Grehs (grehs@inf.pucrs.br)
# DATE CREATED: 19/04/06
# FILENAME: bootTask.asm
# PROJECT: Plasma CPU core
# DESCRIPTION:
#    Initializes the stack pointer and jumps to main().
#    Handles the syscall.
##################################################################
        .text
        .align  2
        .globl  entry
        .ent    entry
entry:
   .set noreorder
   lui     $sp,0
   ori     $sp,$sp, 0x4000     #initialize stack pointer
   la      $gp,_gp
   jal   main
   nop
   
   move $4,$0   
   syscall 
   nop
   
$L1:
   j $L1
   nop

        .end entry
  
###################################################

   .globl SystemCall
   .ent SystemCall
SystemCall:
   .set	noreorder
   
   syscall 
   nop
   jr	$31
   nop
   
   .set reorder
   .end SystemCall

