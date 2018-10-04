/*--------------------------------------------------------------------
 * TITLE: Plasma Real Time Operating System
 * AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
 * DATE CREATED: 12/17/05
 * FILENAME: plasma.h
 * PROJECT: Plasma CPU core
 * COPYRIGHT: Software placed into the public domain by the author.
 *    Software 'as is' without warranty.  Author liable for nothing.
 * DESCRIPTION:
 *    Plasma Hardware Defines
 *--------------------------------------------------------------------*/
#ifndef __PLASMA_H__
#define __PLASMA_H__

/*********** Hardware addresses ***********/
#define UART_WRITE        	0x20000000
#define UART_READ         	0x20000000
#define IRQ_MASK          	0x20000010
#define IRQ_STATUS        	0x20000020
#define COUNTER_REG       	0x20000060
#define SYS_CALL		   	0x20000070
#define END_SIM 		   	0x20000080

/* Network Interface*/
#define	NI_STATUS_RECV		0x20000100
#define	NI_STATUS_SEND		0x20000110
#define	NI_RECV				0x20000120
#define	NI_SEND				0x20000130
#define	NI_CONFIG			0x20000140
#define	NI_ACK				0x20000150
#define	NI_NACK				0x20000160
#define	NI_END				0x20000170
#define	CURRENT_PAGE		0x20000180
#define NEXT_PAGE			0x20000190
#define	NI_SEEK				0x20000260
#define	FAILED_RCV          0x20000270
#define	RELEASE_NI          0x20000280

/* DMA*/
#define DMA_SIZE		  	0x20000200
#define DMA_ADDRESS		  	0x20000210
#define DMA_OP			  	0x20000220
#define DMA_START		  	0x20000230
#define DMA_ACK			  	0x20000240
#define DMA_ACTIVE	  		0x20000250
#define DMA_TARGET          0x20000390
#define DMA_TASK_ID         0x20000400

/* DMA operations */
#define READ	0
#define WRITE	1

#define TICK_COUNTER	  	0x20000300

#define REQ_TASK	  		0x20000350
#define ACK_TASK	  		0x20000360
#define SEEK_REG	  		0x20000370
#define CLEAR_REG	  		0x20000380
#define SEEK_SERVICE_REG    0x20000390

/*********** Interrupt bits **************/
#define IRQ_RESEND_M   			 0x01
#define IRQ_UART_WRITE_AVAILABLE 0x02
#define IRQ_COUNTER18_NOT        0x04
#define IRQ_COUNTER18            0x08
#define IRQ_DMA					 0x10
#define IRQ_NOC					 0x20
#define IRQ_SYS_CALL             0x40 
#define IRQ_SEEK			     0x80 
         
/*Memory Access*/
#define MemoryRead(A) (*(volatile unsigned int*)(A))
#define MemoryWrite(A,V) *(volatile unsigned int*)(A)=(V)

/*--------------------------------------------------------------------
* NI_Read
*
* DESCRIPTION:
*    Reads a 32 bits word from the network interface.
*
*--------------------------------------------------------------------*/
/*#######################
#######WARNING########### MISSES INLINE
#######################*/
unsigned int NI_Read() {

	while(!MemoryRead(NI_STATUS_RECV));

	return MemoryRead(NI_RECV);
}

unsigned int NI_Read_ft(int *status){

    if(*status == 0){//if there is no previous error in NI_Read
        while(!MemoryRead(NI_STATUS_RECV)){
            if(MemoryRead(FAILED_RCV) == 1){//if there is failed packet
                *status = 1;//set status to true
                return 0;//return any value
            }
        }
        return MemoryRead(NI_RECV);
    }
    else{//if there is previous error, do not read nothing from NI
        return 0;
    }
}


/*--------------------------------------------------------------------
* NI_Write
*
* DESCRIPTION:
*    Writes a 32 bits word to the network interface.
*
*--------------------------------------------------------------------*/
/*#######################
#######WARNING########### MISSES INLINE
#######################*/
void NI_Write(unsigned int data) {

	while(!MemoryRead(NI_STATUS_SEND));

	MemoryWrite(NI_SEND,data);
}

#endif /*__PLASMA_H__*/

