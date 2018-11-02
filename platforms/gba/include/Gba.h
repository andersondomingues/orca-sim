#ifndef __GBA_H
#define __GBA_H

/** CONFIGURATIONS **/
#define MEM_PAK_SIZE   0x00200000 /*TODO: set real size */
#define MEM_CART_SIZE  0x00200000 /*TODO: set real size */

/** MEM MAPPING
- System ROM (0x00000000 - 0x00003FFF) 14kb   32-bit word
- EW RAM     (0x02000000 - 0x0203FFFF) 256kb  16-bit word
- IW RAM     (0x03000000 - 0x03007FFF) 32kb   32-bit word
- IO RAM     (0x04000000 - 0x040003FF) 1kb    16-bit word
- PAL RAM    (0x05000000 - 0x050003FF) 1kb    16-bit word
- VRAM       (0x06000000 - 0x06017FFF) 96kb   16-bit word
- OAM        (0x07000000 - 0x070003FF) 1kb    32-bit word
- PAK ROM    (0x08000000 -    ????   ) ????   16-bit word   
- Cart RAM   (0x0E000000 -    ????   ) ????    8-bit word */
#define RAM_SYSROM_BASE 0x00000000 
#define RAM_SYSROM_SIZE 0x00003FFF
#define RAM_EW_BASE     0x02000000
#define RAM_EW_SIZE     0x0003FFFF
#define RAM_IW_BASE     0x03000000
#define RAM_IW_SIZE     0x00007FFF
#define RAM_IO_BASE     0x04000000
#define RAM_IO_RAM      0x000003FF
#define RAM_PAL_BASE    0x05000000
#define RAM_PAL_SIZE    0x000003FF
#define RAM_VRAM_BASE   0x06000000
#define RAM_VRAM_SIZE   0x00017FFF
#define RAM_OAM_BASE    0x07000000
#define RAM_OAM_SIZE    0x000003FF
#define RAM_PAK_BASE    0x08000000
#define RAM_PAK_SIZE    (MEM_PAK_SIZE)
#define RAM_CART_BASE   0x0E000000
#define RAM_CART_SIZE   (MEM_CART_SIZE)


#endif /* __GBA_H */