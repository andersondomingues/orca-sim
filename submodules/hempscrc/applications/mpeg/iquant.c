/*---------------------------------------------------------------------
TITLE: Program Scheduler
AUTHOR: Nicolas Saint-jean
EMAIL : saintjea@lirmm.fr
DATE CREATED: 04/04/06
FILENAME: task2.c
PROJECT: Network Process Unit
COPYRIGHT: Software placed into the public domain by the author.
           Software 'as is' without warranty.  Author liable for nothing.
DESCRIPTION: This file contains the task2
---------------------------------------------------------------------*/

// *********************************************************************************************************
// *                      IQUANT.C                                                                         *
// *                                                                                                       *
// * Inverse quantization algorithm for Mpeg and Jpeg decoding                                             *
// * Used to reconstruct an unquantized 8x8 image block                                                    *
// * Note that in order to support a decent VLC step the quantization factor has been chosen 1             *
// * (extremely redundant) while the Jpeg conventional value should be 64                                  *
// *                                                                                                       *
// *  Expected Result ->  short int block[64]=                                                             *
// *                       { 696,164,-52,-16, 30,-16,-19,  8,                                              *
// *                          71, 14,-61, -2,-11,-11,  6, 11,                                              *
// *                         -59,-55, 13, 27,-20, -7, 12,-17,                                              *
// *                          28, 22,  3,  3,-11,  6, 11,-23,                                              *
// *                          -2,-28,-27,  9,  0, -7, 10,  6,                                              *
// *                           6,  5, 19, 20, -9, -8,  3,-15,                                              *
// *                           0, -7,-15,-15,-10, 14, 14, -9,                                              *
// *                           0,  0,  0, 14,  2,-14, -5,  5  };                                           *
// *                                                                                                       *
// *  Exadecimal results: Block -> 02b800a4 ffccfff0 001efff0 ffed0008 0047000e ffc3fffe fff5fff5 0006000b *
// *                               ffc5ffc9 000d001b ffecfff9 000cffef 001c0016 00030003 fff50006 000bffe9 *
// *                               fffeffe4 ffe50009 0000fff9 000a0006 00060005 00130014 fff7fff8 0003fff1 *
// *                               0000fff9 fff1fff1 fff6000e 000efff7 00000000 0000000e 0002fff2 fffb0005 *
// *                                                                                                       *
// *  Number of clock cycles (with these inEcho) -> 4214                                                   *
// *********************************************************************************************************


#include <task.h>
#include <stdlib.h>

typedef int type_DATA; //unsigned

Message msg1;


unsigned char intramatrix[64]={
   8, 16, 19, 22, 26, 27, 29, 34,
  16, 16, 22, 24, 27, 29, 34, 37,
  19, 22, 26, 27, 29, 34, 34, 38,
  22, 22, 26, 27, 29, 34, 37, 40,
  22, 26, 27, 29, 32, 35, 40, 48,
  26, 27, 29, 32, 35, 40, 48, 58,
  26, 27, 29, 34, 38, 46, 56, 69,
  27, 29, 35, 38, 46, 56, 69, 83
};


/* MPEG-2 inverse quantization */
void iquant_function(type_DATA *src, int lx, int dc_prec, int mquant)
{
  int i, j, val, sum, offs;

  offs=0;
  src[0] = src[0] << (3-dc_prec);
  sum = src[0];
  for (j=0; j<8; j++)
  {
    offs  = j * lx;
    for (i=0; i<8;i++)
    {
      if (j|i)
      {
        val = (int)((src[i+offs]*intramatrix[i+j*8]*mquant)>>4);
        src[i+offs] = (val>2047) ? 2047 : ((val<-2048) ? -2048 : val);
				sum += src[i+offs];
      }
    }
  }
  /* mismatch control */
  if ((sum&1)==0)
  src[offs+7] ^= 1;
}

int main()
{
    unsigned int time_a, time_b;
	int i,j;

    type_DATA clk_count;
    type_DATA block[64];


    Echo("MPEG Task C start: iquant ");
    Echo(itoa(GetTick()));

    for(j=0;j<8;j++)
    {
       Receive(&msg1,ivlc);

       for(i=0;i<msg1.length;i++)
            block[i] = msg1.msg[i];

        iquant_function(block, 8, 0, 1);  // 8x8 Blocks, DC precision value = 0, Quantization coefficient (mquant) = 64

        msg1.length = 64;
        for(i=0; i<msg1.length; i++)
            msg1.msg[i] = block[i];

        Send(&msg1,idct);

    }

   Echo(itoa(GetTick()));
   Echo("End Task C- MPEG");

   exit();
}


