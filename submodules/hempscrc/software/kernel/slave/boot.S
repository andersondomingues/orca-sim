##################################################################
# TITLE: Boot Up Code
# AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
#         Ismael Augusto Grehs (grehs@inf.pucrs.br)
# DATE CREATED: 19/04/06
# FILENAME: bootKernel.asm
# PROJECT: Plasma CPU core
# DESCRIPTION:
#    Initializes the stack pointer and jumps to main().
#    Saves and restores the tasks context.
#    Handles the syscall.
##################################################################

        .text      
        .align  2
        .set noat
        .globl  entry
        .ent    entry
entry:
        .set noreorder


		la      $gp,_gp
		lui     $sp,0
        ori     $sp,$sp, 0x7800     #initialize stack pointer
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

#address 0x3c
interrupt_service_routine_entry:  
        j       interrupt_service_routine
        nop

#address 0x44
system_service_routine_entry:  
        j       system_service_routine 
        nop

#address 0x4c
interrupt_service_routine:

        #REGISTERS $26 AND $27 ARE RESERVED FOR THE OS  
        #la      $26,needTaskScheduling
        #sw      $0,0($26)       

        #SAVE ra AND REGISTERS
        la      $27,current
        lw      $27,0($27)
                                   
        sw      $2,0($27)       #  $v0  
        sw      $3,4($27)       #  $v1
        sw      $4,8($27)       #  $a0
        sw      $5,12($27)      #  $a1
        sw      $6,16($27)      #  $a2
        sw      $7,20($27)      #  $a3
        sw      $8,24($27)      #  $t0
        sw      $9,28($27)      #  $t1
        sw      $10,32($27)     #  $t2  
        sw      $11,36($27)     #  $t3
        sw      $12,40($27)     #  $t4
        sw      $13,44($27)     #  $t5
        sw      $14,48($27)     #  $t6
        sw      $15,52($27)     #  $t7
        sw      $16,56($27)     #  $s0  
        sw      $17,60($27)     #  $s1
        sw      $18,64($27)     #  $s2
        sw      $19,68($27)     #  $s3
        sw      $20,72($27)     #  $s4
        sw      $21,76($27)     #  $s5
        sw      $22,80($27)     #  $s6
        sw      $23,84($27)     #  $s7
        sw      $24,88($27)     #  $t8
        sw      $25,92($27)     #  $t9
        sw      $28,96($27)     #  $gp  
        sw      $29,100($27)    #  $sp  
        sw      $30,104($27)    #  $s8
        sw      $31,108($27)    #  $ra  
        mfhi    $26
        sw      $26,112($27)    #  $hi
        mflo    $26
        sw      $26,116($27)    #  $lo
        mfc0    $26,$14                 
        sw      $26,120($27)    #  save PC


        #SET THE GLOBAL POINTER OF THE OS
        la      $27,gp_ptr
        lw      $27,0($27)
        lw      $28,0($27)      

        #SET THE STACK POINTER OF THE OS
        la      $27,sp_ptr
        lw      $27,0($27)
        lw      $29,0($27) 
        
        #JUMP TO INTERRUPT SERVICE ROUTINE
        lui     $5,0x2000    
        lw      $4,0x20($5)     #IRQ_STATUS
        lw      $6,0x10($5)     #IRQ_MASK
        jal     OS_InterruptServiceRoutine
        and     $4,$4,$6

####################################################

system_service_routine:
        # registers $26 and $27 are reserved for kernel       
        la      $27,current
        lw      $27,0($27)
        
        # saves minimun context
        sw      $4,8($27)       # $a0
        sw      $5,12($27)      # $a1
        sw      $6,16($27)      # $a2
        sw      $7,20($27)      # $a3        

        sw      $28,96($27)     # $gp 
        sw      $29,100($27)    # $sp  
        sw      $31,108($27)    # $ra
        
        mfc0    $26,$14                 
        sw      $26,120($27)    # save PC                


        # sets the kernel global pointer
        la      $27,gp_ptr
        lw      $27,0($27)
        lw      $28,0($27)

        # sets the kernel stack pointer
        la      $27,sp_ptr
        lw      $27,0($27)        
        lw      $29,0($27)

        # jumps to system calls handler
        jal     Syscall
        nop
        
        
        # verifies if scheduling is needed
        la      $26,bug_syscall
        lw      $26,0($26)
        beqz    $26,system_service_restore
        nop
        
        # saves the context of the running task
        la      $4,current
        lw      $4,0($4)
        jal     ASM_SaveRemainingContext
        nop
        
        # schedules the next ready task
        jal     Scheduler
        nop
        
        # restores the context and runs the scheduled task
        la      $4,current
        lw      $4,0($4)
        jal     ASM_RunScheduledTask        
        nop
       
system_service_restore:

        la      $27,current
        lw      $27,0($27)        
        
        # restores the context previously saved
        lw      $28,96($27)     # $gp 
        lw      $29,100($27)    # $sp  
        lw      $31,108($27)    # $ra
        
        lw      $26,120($27)    # loads pc of the task that was running
        lw      $27,124($27)    # loads offset of the task (for paging setup)
        li      $1,0x1  
        
        mtc0    $27,$10
        jr      $26             # jumps to pc
        mtc0    $1,$12          # enables interrupts    

        .set    reorder 
        .end    entry
        
###################################################
        
        .globl  ASM_SaveRemainingContext        
        .ent    ASM_SaveRemainingContext
        .set    noreorder
        
ASM_SaveRemainingContext:

        sw      $2,0($4)        #  $v0  
        sw      $3,4($4)        #  $v1
        #sw      $8,24($4)       #  $t0
        #sw      $9,28($4)       #  $t1
        #sw      $10,32($4)      #  $t2  
        #sw      $11,36($4)      #  $t3
        #sw      $12,40($4)      #  $t4
        #sw      $13,44($4)      #  $t5
        #sw      $14,48($4)      #  $t6
        #sw      $15,52($4)      #  $t7
        sw      $16,56($4)      #  $s0  
        sw      $17,60($4)      #  $s1
        sw      $18,64($4)      #  $s2
        sw      $19,68($4)      #  $s3
        sw      $20,72($4)      #  $s4
        sw      $21,76($4)      #  $s5
        sw      $22,80($4)      #  $s6
        sw      $23,84($4)      #  $s7
        #sw      $24,88($4)      #  $t8
        #sw      $25,92($4)      #  $t9
        sw      $30,104($4)     #  $s8
        #mfhi    $26
        #sw      $26,112($4)     #  $hi
        #mflo    $26
        #sw      $26,116($4)     #  $lo
        
        #mfc0    $26,$14                 
        #sw      $26,120($4)     #  save PC
        
        jr      $31
        nop
        
        .set    reorder
        .end    ASM_SaveRemainingContext
        
###################################################

        .globl  ASM_RunScheduledTask
        .ent    ASM_RunScheduledTask
        .set    noreorder
        
ASM_RunScheduledTask:
        
        move    $27,$4
        
        lw      $2,0($27)       # $v0  
        lw      $3,4($27)       # $v1
        lw      $4,8($27)       # $a0
        lw      $5,12($27)      # $a1
        lw      $6,16($27)      # $a2
        lw      $7,20($27)      # $a3
        lw      $8,24($27)      # $t0
        lw      $9,28($27)      # $t1
        lw      $10,32($27)     # $t2  
        lw      $11,36($27)     # $t3
        lw      $12,40($27)     # $t4
        lw      $13,44($27)     # $t5
        lw      $14,48($27)     # $t6
        lw      $15,52($27)     # $t7
        lw      $16,56($27)     # $s0  
        lw      $17,60($27)     # $s1
        lw      $18,64($27)     # $s2
        lw      $19,68($27)     # $s3
        lw      $20,72($27)     # $s4
        lw      $21,76($27)     # $s5
        lw      $22,80($27)     # $s6
        lw      $23,84($27)     # $s7
        lw      $24,88($27)     # $t8
        lw      $25,92($27)     # $t9
        lw      $28,96($27)     # $gp  
        lw      $29,100($27)    # $sp  
        lw      $30,104($27)    # $s8
        lw      $31,108($27)    # $ra  
        lw      $26,112($27)    # $hi
        mthi    $26
        lw      $26,116($27)    # $lo
        mtlo    $26

        lw      $26,120($27)    # loads pc of the task that will run
        lw      $27,124($27)    # loads offset of the task (for paging setup)
        li      $1,0x1  

        mtc0    $27,$10
        jr      $26             # jumps to pc
        mtc0    $1,$12          # enables interrupts
        
        .set    reorder
        .end    ASM_RunScheduledTask
        
###################################################

        .globl  ASM_SetInterruptEnable
        .ent    ASM_SetInterruptEnable
        
ASM_SetInterruptEnable:
        .set    noreorder
        mfc0    $2,$12
        jr      $31
        mtc0    $4,$12          #enable interrupts

        .set    reorder
        .end    ASM_SetInterruptEnable
        
################## PUTS ##################
        .globl  puts
        .ent    puts
        .set    noreorder
puts:       
        li      $9, 0x20000000  #this is the address of UART_WRITE defined in plasma.h file
        move    $8, $4          # $4 is the first argument
      
loop_puts:
        lw      $10, 0($8)      # read 4 consecutive bytes
        sw      $10, 0($9)      # and store them in the UART address 
        
        lbu     $10, 0($8)      # test the end of the string
        beqz    $10, fim_puts
        lbu     $10, 1($8)
        beqz    $10, fim_puts
        lbu     $10, 2($8)
        beqz    $10, fim_puts
        lbu     $10, 3($8)
        beqz    $10, fim_puts
         
        j        loop_puts
        addiu   $8, $8, 4
        
fim_puts:
        jr      $31             # return to the caller
        nop
        
        .set    reorder
        .end    puts
