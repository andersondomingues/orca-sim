#include "ram_plasma.h"

#ifdef MTI_SYSTEMC
SC_MODULE_EXPORT(ram_plasma);
#endif


/*** Memory read port A ***/
void ram_plasma::read_a() {

	unsigned int address;

	address = (unsigned int)address_a.read();

	if ( address < RAM_SIZE )
		data_read_a.write(ram[address]);
}


/*** Memory write port A ***/
void ram_plasma::write_a() {

	unsigned int data, address;
	unsigned char wbe;

	wbe = (unsigned char)wbe_a.read();
	address = (unsigned int)address_a.read();


	if ( wbe != 0 && address < RAM_SIZE) {
		data = ram[address];

		switch(wbe) {
			case 0xF:	// Write word
				ram[address] = data_write_a.read();
			break;

			case 0xC:	// Write MSW
				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);
			break;

			case 3:		// Write LSW
				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);
			break;

			case 8:		// Write byte 3
				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);
			break;

			case 4:		// Write byte 2
				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);
			break;

			case 2:		// Write byte 1
				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);
			break;

			case 1:		// Write byte 0
				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);
			break;
		}
	}
}


/*** Memory read port B ***/
void ram_plasma::read_b() {

	unsigned int address;

	address = (unsigned int)address_b.read();

	if ( address < RAM_SIZE )
		data_read_b.write(ram[address]);
}


/*** Memory write port B ***/
void ram_plasma::write_b() {

	unsigned int data, address;
	unsigned char wbe;

	wbe = (unsigned char)wbe_b.read();
	address = (unsigned int)address_b.read();


	if ( wbe != 0 && address < RAM_SIZE) {
		data = ram[address];

		switch(wbe) {
			case 0xF:	// Write word
				ram[address] = data_write_b.read();
			break;

			case 0xC:	// Write MSW
				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);
			break;

			case 3:		// Write LSW
				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);
			break;

			case 8:		// Write byte 3
				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);
			break;

			case 4:		// Write byte 2
				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);
			break;

			case 2:		// Write byte 1
				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);
			break;

			case 1:		// Write byte 0
				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);
			break;
		}
	}
}

