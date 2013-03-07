#ifndef __HX_711_H__
#define __HX_711_H__

#include "cc253x.h"

#define ADDO P1_5
#define ADSK P1_4

int ReadCount(void);


int ReadCount(void)
{
 unsigned long Count;
 int ADval;
 unsigned char i,j;

// ADDO=1;
// ADSK=0;
 //clock_delay_usec(10);
 Count=0;
 while(ADDO);
 for(i=0;i<24;i++)
 {
  ADSK=1;
  clock_delay_usec(10);
  Count=Count<<1;
  ADSK=0;
  clock_delay_usec(10);
    if(ADDO)Count++;
 }
 ADSK=1;
// Count=Count^0x800000;
 clock_delay_usec(10);
 ADSK=0;
/* clock_delay_usec(10);
 ADSK=1;
 clock_delay_usec(10);
 ADSK=0;*/
/* if((Count & 0x800000) == 0x800000)
 {
  Count = ~(Count - 1);
 }    */
// ADval = (int)(Count >> 8);//取高十六位有效值
 ADval = (int)(Count>>8);
// ADDO=1;
 return(ADval);
}

#endif
