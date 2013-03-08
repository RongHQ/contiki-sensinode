/**
 * \file
 *         Basic hello world example
 * \author
 *         Zach Shelby <zach@sensinode.com>
 */

#include "contiki.h"
#include "debug.c"
#include "dev/adc-sensor.h"
#include "dev/leds.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
extern process_event_t ADC_sensor_event;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{

  PROCESS_BEGIN();

  printf("Hello World!\n");
  ADC_SENSOR_ACTIVATE();

  while(1){
	  PROCESS_WAIT_EVENT();
	  if(ev == ADC_sensor_event){
		  puthex(ADCL);
		  puthex(ADCH);
		  putchar(',');
		  leds_toggle(LEDS_GREEN);
	  }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
