/**
 * \file
 *         Basic hello world example
 * \author
 *         Zach Shelby <zach@sensinode.com>
 */

#include "contiki.h"
#include "dev/leds.h"

#include "dev/adc-sensor.h"
#include "dev/DS18b20.h"
#include "dev/TM7709.h"
#include "dev/timer4.h"
#include "dev/uart0.h"
#include "dev/serial-line.h"
#include "dev/button-sensor.h"

#include <stdio.h> /* For printf() */



/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/



PROCESS_THREAD(hello_world_process, ev, data)
{

  static struct etimer et;
  static struct sensors_sensor * sensor;
  static int light;
  static unsigned char PWM_value=248;
  int button_status;
  uchar TempInt,TempDot,SignedFlag;

  PROCESS_BEGIN();

  leds_init();

  etimer_set(&et, 64);

  printf("Hello World!\n");
  ADC_SENSOR_ACTIVATE();
  sensor = sensors_find(ADC_SENSOR);
  if (sensor) {
      printf("Sensor OK!\n");
      leds_on(LEDS_GREEN);

  }
  TM7709_init();
  PWM_init();

  while(1){
	  PROCESS_YIELD();
	  if(0/*etimer_expired(&et)*/){
		  leds_invert(0xFF);
		  PWM_value=PWM_value+8;
		  PWM_change(1, PWM_value);

		  light = 2047-(sensor->value(ADC_SENSOR_TYPE_LIGHT));
		  if(light<0) light = 2048;
		  printf("LIGHT:%d\n",light);
	//      ReadCount();
		  printf("Ext ADC:%d\n",TM7709_read_count());
		  ReadTemp(&TempInt,&TempDot,&SignedFlag);
		  printf("DS18b20:%c%d.%d\n",SignedFlag?'-':' ',TempInt,TempDot);
		  printf("PWM:%d\n\r\n",PWM_value);

		  etimer_reset(&et);
	  }
	  if(ev == serial_line_event_message){
		  printf("%s, rxed \n",(char *)data);

	  }
	  if(ev == sensors_event && data == &button_sensor){
		  //printf("%d:",(button_status=button_sensor.value(5)));
		  if(button_status&BUTTON_MASK){
			  printf("A1,56");
		  }
		  if(button_status&BUTTON2_MASK){
		  	  printf("A1,89");
		  }
		  if(button_status&BUTTON_IR_MASK){
		 	  printf("IR.");
		  }
	  }
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
