#ifndef __TM_7709_H__
#define __TM_7709_H__

#include "cc253x.h"

#define DIO  P1_5
#define CLK  P1_4
#define DRDY read_DRDY()

unsigned char read[3]={0,0,0};


void
start(){
	DIO=0;
	clock_delay(1);
	CLK=0;
	clock_delay(1);
}

void
stop(){
	DIO=0;
	clock_delay(1);
	CLK=1;
	clock_delay(1);
	DIO=1;
	clock_delay(1);
}

void
write_byte(unsigned char ONE_BYTE){
	unsigned char i;
	for(i=0;i<8;i++) {
		DIO=(ONE_BYTE&0x80);
		clock_delay(1);
		CLK=1;
		ONE_BYTE<<=1;
		clock_delay(1);
		CLK=0;
		clock_delay(1);
	}

}

void
read_byte(unsigned char n_BYTE) {
	unsigned char i,j,k;
	P1DIR &= ~0x20;
	DIO=1;
	for(j=0;j<n_BYTE;j++) {
		read[j]=0;
		k=0x80;
			for(i=0;i<8;i++) {
				CLK=1;
				clock_delay(1);
				if(DIO) read[j]=read[j]|k;
				k>>=1;
				CLK=0;
				clock_delay(1);
			}
	}
	P1DIR |= (0x20|0x10);
	DIO=0;
	clock_delay(1);
}

void
TM7709_init(){
	P1SEL &= ~(0x20|0x10);
	P1DIR |= (0x20|0x10);
	DIO = 1;
	CLK = 1;

	start();
	write_byte(0xbf);
	write_byte(0x00);
	stop();
}

char
read_DRDY(){
	start();
	write_byte(0x3f);
	DIO=1;
	read_byte(1);
	stop();
	return read[0];
}


int
TM7709_read_count(void)
{
	char timeout=2;
	while(DRDY){
		clock_wait(1);
		if((--timeout)<0){
			return -128;
		}
	}
	start();
	write_byte(0x7f);
	read_byte(3);
	stop();
	return ((read[0]<<8|read[1])-27000);
}

#endif
