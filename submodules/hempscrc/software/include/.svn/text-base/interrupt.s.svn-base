	.text
puta:

	swi	r18, r0, reg	#/*saves r18, which is used to index the others savings*/
	la r18, r0, current	#/*      define o endereço que aponta para a TCB atual */
	lw r18, r0, r18		#/*    lê o endereço da TCB ATUAL */
	swi r3, r18, 0
	swi r4, r18, 4
	swi r5, r18, 8 
	swi r6, r18, 12
	swi r7, r18, 16
	swi r8, r18, 20
	swi r9, r18, 24
	swi r10,r18, 28
	swi r1, r18, 32
	swi r2, r18, 36
	swi r11, r18, 40
	swi r12, r18, 44
	swi r13, r18, 48
	swi r14, r18, 52
	swi r15, r18, 56
	swi r16, r18, 60
	swi r17, r18, 64
	#swi r18, r18,xxxx
	swi r19, r18, 68
	#swi r20, r18,xxxx
	swi r21, r18, 76
	swi r22, r18, 80
	swi r23, r18, 84
	swi r24, r18, 88
	swi r25, r18, 92
	swi r26, r18, 96
	swi r27, r18, 100
	swi r28, r18, 104
	swi r29, r18, 108
	swi r30, r18, 112
	swi r31, r18, 116

	brlid r15, OS_InterruptServiceRoutine
	or	r0, r0, r0
    
