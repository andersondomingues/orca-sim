/*---------------------------------------------------------------------
TITLE: Program Scherduler
AUTHOR: Nicolas Saint-jean
EMAIL : saintjea@lirmm.fr
DATE CREATED: 04/04/06
FILENAME: task1.c
PROJECT: Network Process Unit
COPYRIGHT: Software placed into the public domain by the author.
           Software 'as is' without warranty.  Author liable for nothing.
DESCRIPTION: This file contains the task1
---------------------------------------------------------------------*/

// *********************************************************************************************************
// *                      IVLC.C                                                                           *
// *                                                                                                       *
// * Variable length decoding for Jpeg and Mpeg Compression Algorithms                                     *
// * Used to decode an encoded stream into image blocks (Jpeg) or inter-block differeces (Mpeg)            *
// *                                                                                                       *
// *  Expected Result ->  short int block[64]=                                                             *
// *                       { 87,164,-43,-11, 19, -9,-10,  4,                                               *
// *                         71, 14,-44, -1, -6, -6,  3,  5,                                               *
// *                        -49,-40,  8, 16,-11, -3,  6, -7,                                               *
// *                         21, 16,  2,  2, -6,  3,  5, -9,                                               *
// *                         -1,-17,-16,  5,  0, -3,  4,  2,                                               *
// *                          4,  3, 11, 10, -4, -3,  1, -4,                                               *
// *                          0, -4, -8, -7, -4,  5,  4, -2,                                               *
// *                          0,  0,  0,  6,  1, -4, -1,  1, };                                            *
// *                                                                                                       *
// *  Exadecimal results: Block -> 005700a4 ffd5fff5 0013fff7 fff60004 0047000e ffd4ffff fffafffa 00030005 *
// *                               ffcfffd8 00080010 fff5fffd 0006fff9 00150010 00020002 fffa0003 0005fff7 *
// *                               ffffffef fff00005 0000fffd 00040002 00040003 000b000a fffcfffd 0001fffc *
// *                               0000fffc fff8fff9 fffc0005 0004fffe 00000000 00000006 0001fffc ffff0001 *
// *                                                                                                       *
// *  Number of clock cycles (with these inEcho) -> 57026                                                  *
// *********************************************************************************************************

#include <task.h>
#include <stdlib.h>

typedef int type_DATA; //unsigned


int out;

short int bytecount = 0;              // Bytes of the compressed VLC_array bitstream: must be specified as an input

/*short int block[64]=  // Extracted Image: Must match with the above described result
  {
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0
    };*/

/* zig-zag scan
   Scanning order: The zig-zag scan is used in order to maximize adjacent zeroes */

unsigned char zig_zag_scan[64] =
{
  0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,
  12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
  35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,
  58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
};

typedef struct {
  char val, len;
} VLCtab;

typedef struct {
  char run, level, len;
} DCTtab;


/* Table B-12, dct_dc_size_luminance, codes 00xxx ... 11110 */
static VLCtab DClumtab0[32] =
{ {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2},
  {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2},
  {0, 3}, {0, 3}, {0, 3}, {0, 3}, {3, 3}, {3, 3}, {3, 3}, {3, 3},
  {4, 3}, {4, 3}, {4, 3}, {4, 3}, {5, 4}, {5, 4}, {6, 5}, {0, 0}
};

/* Table B-12, dct_dc_size_luminance, codes 111110xxx ... 111111111 */
static VLCtab DClumtab1[16] =
{ {7, 6}, {7, 6}, {7, 6}, {7, 6}, {7, 6}, {7, 6}, {7, 6}, {7, 6},
  {8, 7}, {8, 7}, {8, 7}, {8, 7}, {9, 8}, {9, 8}, {10,9}, {11,9}
};

/* Table B-13, dct_dc_size_chrominance, codes 00xxx ... 11110 */
static VLCtab DCchromtab0[32] =
{ {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2},
  {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2},
  {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2}, {2, 2},
  {3, 3}, {3, 3}, {3, 3}, {3, 3}, {4, 4}, {4, 4}, {5, 5}, {0, 0}
};

/* Table B-13, dct_dc_size_chrominance, codes 111110xxxx ... 1111111111 */
static VLCtab DCchromtab1[32] =
{ {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6},
  {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6}, {6, 6},
  {7, 7}, {7, 7}, {7, 7}, {7, 7}, {7, 7}, {7, 7}, {7, 7}, {7, 7},
  {8, 8}, {8, 8}, {8, 8}, {8, 8}, {9, 9}, {9, 9}, {10,10}, {11,10}
};


/* Table B-15, DCT coefficients table one,
 * codes 000001xx ... 11111111
*/
DCTtab DCTtab0a[252] =
{
  {65,0,6}, {65,0,6}, {65,0,6}, {65,0,6}, /* Escape */
  {7,1,7}, {7,1,7}, {8,1,7}, {8,1,7},
  {6,1,7}, {6,1,7}, {2,2,7}, {2,2,7},
  {0,7,6}, {0,7,6}, {0,7,6}, {0,7,6},
  {0,6,6}, {0,6,6}, {0,6,6}, {0,6,6},
  {4,1,6}, {4,1,6}, {4,1,6}, {4,1,6},
  {5,1,6}, {5,1,6}, {5,1,6}, {5,1,6},
  {1,5,8}, {11,1,8}, {0,11,8}, {0,10,8},
  {13,1,8}, {12,1,8}, {3,2,8}, {1,4,8},
  {2,1,5}, {2,1,5}, {2,1,5}, {2,1,5},
  {2,1,5}, {2,1,5}, {2,1,5}, {2,1,5},
  {1,2,5}, {1,2,5}, {1,2,5}, {1,2,5},
  {1,2,5}, {1,2,5}, {1,2,5}, {1,2,5},
  {3,1,5}, {3,1,5}, {3,1,5}, {3,1,5},
  {3,1,5}, {3,1,5}, {3,1,5}, {3,1,5},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {1,1,3}, {1,1,3}, {1,1,3}, {1,1,3},
  {64,0,4}, {64,0,4}, {64,0,4}, {64,0,4}, /* EOB */
  {64,0,4}, {64,0,4}, {64,0,4}, {64,0,4},
  {64,0,4}, {64,0,4}, {64,0,4}, {64,0,4},
  {64,0,4}, {64,0,4}, {64,0,4}, {64,0,4},
  {0,3,4}, {0,3,4}, {0,3,4}, {0,3,4},
  {0,3,4}, {0,3,4}, {0,3,4}, {0,3,4},
  {0,3,4}, {0,3,4}, {0,3,4}, {0,3,4},
  {0,3,4}, {0,3,4}, {0,3,4}, {0,3,4},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,1,2}, {0,1,2}, {0,1,2}, {0,1,2},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,2,3}, {0,2,3}, {0,2,3}, {0,2,3},
  {0,4,5}, {0,4,5}, {0,4,5}, {0,4,5},
  {0,4,5}, {0,4,5}, {0,4,5}, {0,4,5},
  {0,5,5}, {0,5,5}, {0,5,5}, {0,5,5},
  {0,5,5}, {0,5,5}, {0,5,5}, {0,5,5},
  {9,1,7}, {9,1,7}, {1,3,7}, {1,3,7},
  {10,1,7}, {10,1,7}, {0,8,7}, {0,8,7},
  {0,9,7}, {0,9,7}, {0,12,8}, {0,13,8},
  {2,3,8}, {4,2,8}, {0,14,8}, {0,15,8}
};

/* Table B-15, DCT coefficients table one,
 * codes 000000100x ... 000000111x
 */
DCTtab DCTtab1a[8] =
{
  {5,2,9}, {5,2,9}, {14,1,9}, {14,1,9},
  {2,4,10}, {16,1,10}, {15,1,9}, {15,1,9}
};

/* Table B-14/15, DCT coefficients table zero / one,
 * codes 000000010000 ... 000000011111
 */
DCTtab DCTtab2[16] =
{
  {0,11,12}, {8,2,12}, {4,3,12}, {0,10,12},
  {2,4,12}, {7,2,12}, {21,1,12}, {20,1,12},
  {0,9,12}, {19,1,12}, {18,1,12}, {1,5,12},
  {3,3,12}, {0,8,12}, {6,2,12}, {17,1,12}
};

/* Table B-14/15, DCT coefficients table zero / one,
 * codes 0000000010000 ... 0000000011111
 */
DCTtab DCTtab3[16] =
{
  {10,2,13}, {9,2,13}, {5,3,13}, {3,4,13},
  {2,5,13}, {1,7,13}, {1,6,13}, {0,15,13},
  {0,14,13}, {0,13,13}, {0,12,13}, {26,1,13},
  {25,1,13}, {24,1,13}, {23,1,13}, {22,1,13}
};

/* Table B-14/15, DCT coefficients table zero / one,
 * codes 00000000010000 ... 00000000011111
 */
DCTtab DCTtab4[16] =
{
  {0,31,14}, {0,30,14}, {0,29,14}, {0,28,14},
  {0,27,14}, {0,26,14}, {0,25,14}, {0,24,14},
  {0,23,14}, {0,22,14}, {0,21,14}, {0,20,14},
  {0,19,14}, {0,18,14}, {0,17,14}, {0,16,14}
};

/* Table B-14/15, DCT coefficients table zero / one,
 * codes 000000000010000 ... 000000000011111
 */
DCTtab DCTtab5[16] =
{
  {0,40,15}, {0,39,15}, {0,38,15}, {0,37,15},
  {0,36,15}, {0,35,15}, {0,34,15}, {0,33,15},
  {0,32,15}, {1,14,15}, {1,13,15}, {1,12,15},
  {1,11,15}, {1,10,15}, {1,9,15}, {1,8,15}
};

/* Table B-14/15, DCT coefficients table zero / one,
 * codes 0000000000010000 ... 0000000000011111
 */
DCTtab DCTtab6[16] =
{
  {1,18,16}, {1,17,16}, {1,16,16}, {1,15,16},
  {6,3,16}, {16,2,16}, {15,2,16}, {14,2,16},
  {13,2,16}, {12,2,16}, {11,2,16}, {31,1,16},
  {30,1,16}, {29,1,16}, {28,1,16}, {27,1,16}
};

short int bitposition = 7;            // posizione del prossimo bit da scrivere sul buffer
short int mask = 0x80;                // mask for reading and writing bytes on the vlc_stream


// ************************************************************************
//  IVLC SubFunctions ...                                                 *
// ************************************************************************

unsigned int getbits (short int n, short int flush, type_DATA *buffer, short int init)
{
  short int i = 0;
  short int temp = 0;
  int temp2 = 0;
  short int tmp_bytecnt = bytecount;
  short int tmp_bitpstn  = bitposition;
  unsigned char tmp_mask = mask;

  //printfuart("start getbits:");
  //printfuart("buffer:"); printfuart(itoa(buffer));
  //printfuart("tmp_bytecnt:"); printfuart(itoa(tmp_bytecnt));
  //printfuart("tmp_mask:"); printfuart(itoa(tmp_mask));
  //printfuart("tmp_bitpstn:"); printfuart(itoa(tmp_bitpstn));
  if(init == 0)
    {
      for (i = 0; i < n; i++) {
	temp = (buffer [tmp_bytecnt] & tmp_mask) >> tmp_bitpstn;
	temp2 <<= 1;
	temp2 += temp;
	tmp_mask >>= 1;
	tmp_bitpstn -= 1;
	if (tmp_bitpstn == -1) {
	  tmp_bitpstn = 7;
	  tmp_bytecnt += 1;
	  tmp_mask = 0x80;
	}
      }
      //printfuart("temp:"); printfuart(itoa(temp));
      //printfuart("temp2:"); printfuart(itoa(temp2));
      //printfuart("tmp_mask:"); printfuart(itoa(tmp_mask));
      //printfuart("tmp_bitpstn:"); printfuart(itoa(tmp_bitpstn));

      if (flush) {
	bytecount = tmp_bytecnt;
	bitposition = tmp_bitpstn;
	mask = tmp_mask;
      }
      //printfuart("End getbits:");
    }
  else
    {
      //printfuart("Init");
      tmp_bytecnt = 0;
      bytecount = 0;
      tmp_bitpstn  = 7;
      bitposition = 7;
      tmp_mask = 0x80;
      mask =0x80;
      //printfuart("tmp_bytecnt:"); printfuart(itoa(tmp_bytecnt));
      //printfuart("tmp_mask:"); printfuart(itoa(tmp_mask));
      //printfuart("tmp_bitpstn:"); printfuart(itoa(tmp_bitpstn));
    }
  return temp2;
}

/* unsigned int getbits (short int n, short int flush, type_DATA *buffer, short int init) */
/* { */
/*   short int i = 0; */
/*   short int temp = 0; */
/*   int temp2 = 0; */
/*   short int tmp_bytecnt = bytecount; */
/*   short int tmp_bitpstn = bitposition; */
/*   unsigned char tmp_mask = mask; */
/*   //printfuart(""); */
/*   for (i = 0; i < n; i++) { */
/*     temp = (buffer [tmp_bytecnt] & tmp_mask) >> tmp_bitpstn; */
/*     temp2 <<= 1; */
/*     temp2 += temp; */
/*     tmp_mask >>= 1; */
/*     tmp_bitpstn -= 1; */
/*     if (tmp_bitpstn == -1) { */
/*       tmp_bitpstn = 7; */
/*       tmp_bytecnt += 1; */
/*       tmp_mask = 0x80; */
/*     } */
/*   } */

/*   if (flush) { */
/*     bytecount = tmp_bytecnt; */
/*     bitposition = tmp_bitpstn; */
/*     mask = tmp_mask; */
/*   } */
/*   //printfuart(""); */
/*   return temp2; */
/* } */

short int getDC (short int type, type_DATA *buffer)
{
  short int code, size, val;

  code = getbits (5, 0, buffer,0);
  //printfuart("codeDC:"); printfuart(itoa(code));

  if (code < 31) {
    if (type) {
      size = DCchromtab0[code].val;
      getbits (DCchromtab0[code].len, 1, buffer,0);
    }
    else {
      size = DClumtab0 [code].val;
      getbits (DClumtab0 [code].len, 1, buffer,0);
    }
  }
  else {
    if (type) {
      code = getbits (10, 0, buffer,0) - 0x3E0;
      size = DCchromtab1 [code].val;
      getbits (DCchromtab1 [code].len, 1, buffer,0);
    }
    else {
      code = getbits (9, 0, buffer,0) - 0x1F0;
      size = DClumtab1 [code].val;
      getbits (DClumtab1 [code].len, 1, buffer,0);
    }
  }

   if (size == 0) {
    val = 0;
  }
  else {
    val = getbits (size, 1, buffer,0);
    if ((val & (1 << (size - 1))) == 0)
      val -= (1 << size) - 1;
  }
  return val;
}


// ************************************************************************************
// * IVLC Function
// ************************************************************************************

void ivlc_function (type_DATA *block, short int comp, short int lx, type_DATA *buffer)
{
  int val, i, sign, run;
  short int temp, temp2;
  unsigned int code;
  DCTtab *tab;

  sign = getbits(0,0,0,1);      /*init get bit*/

  /* decode DC coefficients */
  if (comp == 0) {
    block[0] = getDC(0, buffer);
    //printfuart("block[0]:"); printfuart(itoa(block[0]));
  }
  else if (comp == 1) {
    block[0] = getDC(1, buffer);
  }
  else {
    block[0] = getDC(1, buffer);
  }

   /* decode AC coefficients */
  for (i=1; ; i++)
  {
    code = getbits (16, 0, buffer,0);
    //printfuart("codeAC:"); printfuart(itoa(code));
    if (code>=1024)
      tab = &DCTtab0a[(code>>8)-4];
    else if (code>=512)
      tab = &DCTtab1a[(code>>6)-8];
    else if (code>=256)
      tab = &DCTtab2[(code>>4)-16];
    else if (code>=128)
      tab = &DCTtab3[(code>>3)-16];
    else if (code>=64)
      tab = &DCTtab4[(code>>2)-16];
    else if (code>=32)
      tab = &DCTtab5[(code>>1)-16];
    else if (code>=16)
      tab = &DCTtab6[code-16];
    else
      return;

    getbits (tab->len, 1, buffer,0);

    if (tab->run==64) /* end_of_block */
      return;

    if (tab->run==65) /* escape */
    {
      i += run = getbits(6, 1, buffer,0);
      val = getbits(12, 1, buffer,0);
      if ((val&2047)==0)
        return;
      if (sign = val>=(2048))
        val = 4096 - val;
    }
    else
    {
      i+= run = tab->run;
      val = tab->level;
      sign = getbits(1, 1, buffer,0);
    }

    if (i>=64)
    {
      return;
    }

    temp = zig_zag_scan[i] >> 3;
    temp2 = (zig_zag_scan[i] - (temp << 3));

    block[(temp * lx + temp2)] = sign ? -val : val;
  }
  return;
}

Message msg1;

int main()
{
	unsigned int time_a, time_b;
	int i,j;

	type_DATA vlc_array[128];
	type_DATA block[64];

	Echo("MPEG Task B start: iVLC ");
	Echo(itoa(GetTick()));

	for(j=0;j<8;j++)
	{

		Receive(&msg1,start);

		for(i=0; i<msg1.length; i++)
			vlc_array[i] = msg1.msg[i];

		for(i=0; i<64; i++)
			block[i] = 0;

		ivlc_function(block, 0, 8, vlc_array);	    // codifica RLE-VLC (returns the number of bits in the produced stream)

		msg1.length = 64;
		for(i=0; i<msg1.length; i++)
		   msg1.msg[i] = block[i];

        Send(&msg1,iquant);

	}

	Echo(itoa(GetTick()));
	Echo("End Task B - MPEG");

	exit();
}
