#ifndef __TIMER_4_H__
#define __TIMER_4_H__

#include "cc253x.h"
#include "contiki-conf.h"

void
PWM_init(){
	PERCFG &= ~(0x10);
	P2SEL |= 0x10;
	P1SEL |= (0x01 | 0x02);

	T4CC0 = 0x01;
	T4CC1 = 0x01;

	T4CCTL0 = (0x04 | 0x20);
	T4CCTL1 = (0x04 | 0x20);

	T4CTL = 0x90;
	T4CTL = (0x90 | 0x04);
}

void
PWM_change(char channel, char value){
	switch(channel){
	case 0:

		if(value==0){
			T4CCTL0 = (0x04 | 0x08);
		}
		else{
			T4CCTL0 = (0x04 | 0x20);
			T4CC0 = value;
		}
		break;
	case 1:

		if(value==0){
			T4CCTL1 = (0x04 | 0x08);
		}
		else{
			T4CCTL1 = (0x04 | 0x20);
			T4CC1 = value;
				}
		break;
	default:
		return;
	}

}

#endif
