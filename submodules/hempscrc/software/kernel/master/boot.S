##################################################################
# TITLE: Boot Up Code 
# AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
#         Ismael Augusto Grehs (grehs@inf.pucrs.br)
# DATE CREATED: 19/04/06
# FILENAME: bootKernel.asm
# PROJECT: Plasma CPU core
# DESCRIPTION:
#    Initializes the stack pointer and jumps to main().
##################################################################

        .text    
        .set noat
        .align  2
        .globl  entry
        .ent    entry
entry:
        .set noreorder
		
		la      $gp,_gp
        lui     $sp,0
        ori     $sp,$sp, 0xc800     #initialize stack pointer
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        jal     main
        nop
$L1:
        j $L1
        nop
        
        .end entry
         
