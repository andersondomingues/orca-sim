/*--------------------------------------------------------------------
 * TITLE: Standart functions.
 * AUTHOR: Ismael Augusto Grehs (grehs@inf.pucrs.br)
 * DATE CREATED: 19/07/07
 * FILENAME: stdlib.h
 * PROJECT: HeMPS (Hermes Multiprocessing System)
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------*/

#ifndef __STDLIB_H__
#define __STDLIB_H__

#include <stdarg.h>

#define FALSE		0
#define TRUE		1


char *itoa_base(int num, int base) {

   int digit, negate=0, place;
   char c, text[20];
   static char dst[20];

   if(base == 10 && num < 0) {
      num = -num;
      negate = 1;
   }

   text[16] = 0;
   for(place = 15; place >= 0; --place) {
      digit = (unsigned int)num % (unsigned int)base;
      if(num == 0 && place < 15 && base == 10 && negate) {
         c = '-';
         negate = 0;
      }
      else if(digit < 10)
         c = (char)('0' + digit);
      else
         c = (char)('A' + digit - 10);
      text[place] = c;
      num = (unsigned int)num / (unsigned int)base;
      if(num == 0 && negate == 0)
         break;
   }
   strcpy(dst, text + place);
   return dst;
}

char *itoa(unsigned int num)
{
   static char buf[12];
   static char buf2[12];
   int i,j;

   if (num==0)
   {
      buf[0] = '0';
      buf[1] = '\0';
      return &buf[0];
   }

   for(i=0;i<11 && num!=0;i++)
   {
      buf[i]=(char)((num%10)+'0');
      num/=10;
   }
   buf2[i] = '\0';
   j = 0;
   i--;
   for(;i>=0;i--){
         buf2[i]=buf[j];
         j++;
   }
   return &buf2[0];
}


char *itoh(unsigned int num)
{
   static char buf[11];
   int i;
   buf[10]=0;

   buf[0] = '0';
   buf[1] = 'x';

   if (num==0)
   {
      buf[2] = '0';
      buf[3] = '0';
      buf[4] = '0';
      buf[5] = '0';
      buf[6] = '0';
      buf[7] = '0';
      buf[8] = '0';
      buf[9] = '0';
      return buf;
   }

   for(i=9;i>=2;--i)
   {
      if ((num%16) < 10)
         buf[i]=(char)((num%16)+'0');
      else
         buf[i]=(char)((num%16)+'W');

      num/=16;
   }

   return buf;
}

char * strcat(char *dest, const char *src)
{
    unsigned int i,j;
	static char str[128];
    for (i = 0; dest[i] != '\0'; i++) str[i] = dest[i];
    for (j = 0; src[j] != '\0'; j++)  str[i+j] = src[j];
    str[i+j] = '\0';
    return str;
}

int abs(int num)
{
	if(num<0) return -num;
	else return num;
}

int rand(int seed, int min, int max)
{ 
	int lfsr = seed;
	
	lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
	
	return (lfsr % max + min);
} 

int add(int a, int b)
{
      int res;

      res=a+b;
      return res;
}


int sub(int a, int b)
{
      int res;

      res=a-b;
      return res;
}

char *fixetoa(int nume)
{
 static char buf[13];
 static char buf2[13];
 int i,j;
 int num=nume;
 
 if (num<0)
 {
 num=num*(-1);
       
 for(i=0;i<12 && (num!=0 || i<5) ;i++)
 {
               if (num==0)
                          buf[i]='0';
               else
               {
                          buf[i]=(char)((num%10)+'0');
                          num/=10;
               }
 }
 i++;
 i++;
 buf2[i] = '\0';
 j = 0;
 i--;

 for(;i>=0;i--)
 {

       if(j==4)
       {
            buf2[i]='.';
            i--;
       }              
       buf2[i]=buf[j];
       j++;

 }

 buf2[0]='-';
 return &buf2[0];
 }




 else
 {
 if (num==0)
 {
    buf[0] = '0';
    buf[1] = '\0';
    return &buf[0];
 }

 for(i=0;i<12 && (num!=0 || i<5);i++)
 {
               if (num==0)
                          buf[i]='0';                 
               else
               {
                          buf[i]=(char)((num%10)+'0');
                          num/=10;
               }
 }
 i++;
 buf2[i] = '\0';
 j = 0;
 i--;
 for(;i>=0;i--)
 {

       if(j==4)
       {
            buf2[i]='.';
            i--;
       }              
       buf2[i]=buf[j];
       j++;

 }
 return &buf2[0];
 }
}

void *memset(void *dst, int c, unsigned long bytes) {

   unsigned char *Dst = (unsigned char*)dst;

   while((int)bytes-- > 0)
      *Dst++ = (unsigned char)c;

   return dst;
}

char *strcpy(char *dst, const char *src) {

	char *dstSave=dst;
   	int c;

   	do {
    	c = *dst++ = *src++;
   	} while(c);

   return dstSave;
}


int strlen(const char *string) {

   const char *base=string;

   while(*string++) ;

   return string - base - 1;
}


void memcopy(void *dst, const void *src, unsigned int size) {

	unsigned int i,j;
	unsigned int words = size/4;
	unsigned int bytes = size%4;

	for(i=0; i<words; i++)
		((int*)dst)[i] = ((int*)src)[i];

	for(j=0; j<bytes; j++, i++)
		((char*)dst)[i] = ((char*)src)[i];
}


int sprintf(char *s, const char *format,...) {


   	int width, length;
   	char f, *text, fill;

   	va_list args;

   	va_start(args,format);

   	for(;;) {
      	f = *format++;
      	if(f == 0) {
		  	va_end(args);
         	return 0;
	  	}
      	else if(f == '%') {
         	width = 0;
         	fill = ' ';

         	f = *format++;
         	while('0' <= f && f <= '9') {
            	width = width * 10 + f - '0';
            	f = *format++;
         	}

         	if(f == '.') {
            	fill = '0';
            	f = *format++;
         	}

         	if(f == 0) {
			 	va_end(args);
            	return 0;
		 	}

         	if(f == 'd') {
            	memset(s, fill, width);
            	text = itoa_base(va_arg(args,int),10);
            	length = (int)strlen(text);
            	if(width < length)
               		width = length;
            	strcpy(s + width - length, text);
         	}
         	else if(f == 'x' || f == 'X') {
            	memset(s, '0', width);
            	text = itoa_base(va_arg(args,int),16);
            	length = (int)strlen(text);
            	if(width < length)
               		width = length;
            	strcpy(s + width - length, text);
         	}
         	else if(f == 'c') {
            	*s++ = (char)va_arg(args,int);
            	*s = 0;
         	}
         	else if(f == 's') {
            	text = (char*)va_arg(args,int);
            	length = strlen(text);
            	if(width > length) {
               		memset(s, ' ', width - length);
               		s += width - length;
            	}
            	strcpy(s, text);
         	}

         	s += strlen(s);
      	}
      	else {
        	if(f == '\n')
            	*s++ = '\r';

         	*s++ = f;

         	if(f == '\r' && *format == '\n')
            	*s++ = *format++;
      	}

      	*s = 0;
   	}
}


#endif



