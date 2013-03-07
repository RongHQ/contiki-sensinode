/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <stdio.h>
#include <string.h>
#include "dev/leds.h"
#include "dev/adc-sensor.h"
#include "dev/button-sensor.h"
#include "dev/uart0.h"
#include "dev/DS18b20.h"
#include "dev/TM7709.h"
#include "dev/timer4.h"
//#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		20

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

static char buf[MAX_PAYLOAD_LEN];
static char buf_data[MAX_PAYLOAD_LEN];
static char buf_switch[MAX_PAYLOAD_LEN];

static int button_count[5]={0,0,0,0,0};

/* Our destinations and udp conns. One link-local and one global */
#define LOCAL_CONN_PORT 3002
#define UDP_CLIENT_PORT 3001
#define UDP_SERVER_PORT 3000
static struct uip_udp_conn *server_conn, *switch_conn;
static uip_ipaddr_t server_ipaddr, switch_ipaddr;


/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(temp_process, "temperature process");
AUTOSTART_PROCESSES(&udp_client_process, &temp_process);
/*---------------------------------------------------------------------------*/
static void
send_packet(void)
{
  PRINTF("DATA send to %d 'Hello'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);
  uip_udp_packet_sendto(server_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
  memset(buf, 0, MAX_PAYLOAD_LEN);
  return;
}

PROCESS_THREAD(temp_process, ev, data)
{
  static struct etimer temp_timer;
  uchar TempInt,TempDot,SignedFlag;
  static struct sensors_sensor * sensor;
  static int light_sense;
  static int rv;
  static float voltage = 0;
  static int current=0;
  static float power;
  static int dec;
  static float frac;
 
  int button_status;


  PROCESS_BEGIN();

  PROCESS_PAUSE();


//initialize

  leds_init();

  ADC_SENSOR_ACTIVATE();
  sensor = sensors_find(ADC_SENSOR);
   if (sensor) {
      printf("Sensor OK!\n");
      leds_on(LEDS_GREEN);
      sensor->configure(SENSORS_HW_INIT,1);
  }
  
  TM7709_init();
  PWM_init();
 

  PRINTF("Sensor process started\n");
  ReadTemp(&TempInt, &TempDot, &SignedFlag);

  //the PWM use 0-255,so divide the light sensor value by 8
  light_sense = 2047-(sensor->value(ADC_SENSOR_TYPE_LIGHT));
  if(light_sense<0) light_sense = 2048;
  light_sense=(int)(light_sense/8);
  
  // read the sensor every 3 seconds
  etimer_set(&temp_timer,CLOCK_SECOND * 3);

  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&temp_timer))
        {
          etimer_reset(&temp_timer);
          light_sense = 2047-(sensor->value(ADC_SENSOR_TYPE_LIGHT));
          if(light_sense<0) light_sense = 2048;
          light_sense=(int)(light_sense/8);

          if (button_count[2]==0){
               button_count[3]++;
               if(button_count[3]>5&&button_count[0]==0){ // 5*3 seconds no one
            	   sprintf(buf,"B1,0");
            	   send_packet();
                   PWM_change(1, 0); //switch off
                   PWM_change(0, 0); //switch off
                   button_count[4]=0; //means off
               }
           }
          button_count[2]=0;  // every 3 seconds clear button_count[2]

         if (button_count[0]==0){ //auto
               sprintf(buf,"G1,%d", light_sense);
               send_packet();
           }

          //read the power value
          rv = sensor->value(ADC_SENSOR_TYPE_VDD);
          if(rv != -1) {
             voltage = rv * 3.75 / 2047;
             current= TM7709_read_count();
             power=voltage*current;

             dec = power;
             frac = power - dec;

             if(button_count[4]==0){
                 sprintf(buf,"A1,0");
                 PWM_change(1, 0);
                 PWM_change(0, 0);
                 send_packet();

             }
             else{
                // two decimal places
                  sprintf(buf,"A1,%d.%02u", dec, (unsigned int)(frac*100));
                  send_packet();
              }
           }

          ReadTemp(&TempInt, &TempDot, &SignedFlag);
          printf("DS18b20:%c%d.%d\n",SignedFlag?'-':' ',TempInt,TempDot);
          // may be the temperature should be increased?
          if(TempInt>30){
             sprintf(buf_switch,"C4,1");
             // send to switch mote
             PRINTF("DATA send to %d %s\n",
             switch_ipaddr.u8[sizeof(switch_ipaddr.u8) - 1],buf_switch);
             uip_udp_packet_sendto(switch_conn, buf_switch, strlen(buf_switch),
                            &switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
             PWM_change(0,128);
             memset(buf_switch, 0, MAX_PAYLOAD_LEN);
          }
          leds_toggle(LEDS_RED);
        }

    if(ev == sensors_event && data == &button_sensor) {
         button_status=button_sensor.value(5);
        // if(button_status&BUTTON_MASK)  //auto or manual
	    // button_count[0]=1-button_count[0];

         //if(button_count[0]==0)
          //  PWM_change(0, 255);
         //else
          //  PWM_change(0, 0);

             //button_count[0]==0 means auto, otherwise manual

       /* if(button_status&BUTTON2_MASK){  // manual to switch
	     button_count[1]=1-button_count[1];	
             // button_count[1]==1 means switch on
             if (button_count[0]==1){
                if(button_count[1]==0)
                    PWM_change(1, 0);
                 else
                    PWM_change(1, 255);
              }   
          } */
 
         if(button_status&BUTTON_IR_MASK){  //infrared
             if (button_count[0]==0){ //auto
                  button_count[2]=1; //means there is some one
                  button_count[3]=0; //means the number of finding no one 
                  sprintf(buf,"B1,1");
                   send_packet();
                  if (light_sense<180){  //on when light<180
                      PWM_change(1, 255-light_sense);  
                      PWM_change(0, 255-light_sense);
                      //light stay 255
                      button_count[4]=1; // means on
                   }
                   else {
                	  button_count[4]=0;
                	  PWM_change(1, 0);
                	  PWM_change(0, 0);
                   }
              }
          }
  
     }


  }
  PROCESS_END();
}

static void
tcpip_handler(void)
{
   memset(buf_data, 0, MAX_PAYLOAD_LEN);
   if(uip_newdata()){
        memcpy(buf_data, uip_appdata, uip_datalen());

	    if(buf_data[0] == 'D' && buf_data[3] == '2'){
                button_count[0]=0;
        }
        else if(buf_data[0] == 'D' && buf_data[3] == '1'){
                button_count[0]=1;
                button_count[4]=1;
                PWM_change(1, 255);
                PWM_change(0, 255);
        }
        else if(buf_data[0] == 'D' && buf_data[3] == '0'){
                button_count[0]=1;
                button_count[4]=0;
                PWM_change(1, 0);
                PWM_change(0, 0);
        }
    }
	return;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  PRINTF("UDP client process started\n");
   
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);  //define in uip.h
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr); //define in uip_ds6.c convert mac to ipv6
    uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  uip_ip6addr(&server_ipaddr,0xaaaa,0,0,0,0,0,0,1);
  /* new connection with remote host */
  server_conn = udp_new(&server_ipaddr, UIP_HTONS(0), NULL);
  if(!server_conn) {
    PRINTF("udp_new connenction error.\n");
  }
  udp_bind(server_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Connection with ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
  UIP_HTONS(server_conn->lport), UIP_HTONS(server_conn->rport));

  //!!!!!!!!!!!must be the local address from mac address
  //please test the address of switch mote and replace ?
  uip_ip6addr(&switch_ipaddr, 0xfe80,0,0,0,0x0212,0x4B00,0x019A,0xC600);
  switch_conn = udp_new(&switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL);
 // if(!switch_conn) {
  //  PRINTF("switvh connenction error.\n");
  //}
  udp_bind(switch_conn, UIP_HTONS(LOCAL_CONN_PORT));
  

  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) 
      tcpip_handler();
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
