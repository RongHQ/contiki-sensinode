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
#include <stdlib.h> 

#include "dev/adc-sensor.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/DS18b20.h"
#include "dev/TM7709.h"
#include "dev/timer4.h"
#include "debug.h"

#include "simple-udp.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		20
#define LARGE_POWER               255
#define SMALL_POWER             100

static char buf[MAX_PAYLOAD_LEN];
static char buf_switch[MAX_PAYLOAD_LEN];
static char buf_data[MAX_PAYLOAD_LEN];
static int button_count[5]={0,0,0,0,0};
static int temp_center;

/* Our destinations and udp conns. One link-local and one global */
#define LOCAL_CONN_PORT 3002
#define UDP_CLIENT_PORT 3001
#define UDP_SERVER_PORT 3000
static struct simple_udp_connection server_conn, switch_conn;
static uip_ipaddr_t server_ipaddr, switch_ipaddr;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(wash_process, "washing process");
AUTOSTART_PROCESSES(&udp_client_process, &wash_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(wash_process, ev, data)
{
  static struct etimer temp_timer;
  uchar TempInt,TempDot,SignedFlag;
  static struct sensors_sensor * sensor;
  static int rv;
  static float voltage = 0;
  static int current=0;
  static float power;
  static int dec;
  static float frac;

  int button_status;

  PROCESS_BEGIN();
  
  PROCESS_PAUSE();
   
  ADC_SENSOR_ACTIVATE();
  sensor = sensors_find(ADC_SENSOR);
   if (sensor) {
      printf("Sensor OK!\n");
      leds_on(LEDS_GREEN);
      sensor->configure(SENSORS_HW_INIT,1);
  }
  
  TM7709_init();
  PWM_init(); 
  // the button_count[3] counts the number of not being pressed(3s/count)
  // the button_count[4] counts the number of being pressed 

  PRINTF("Sensor process started\n");
  ReadTemp(&TempInt, &TempDot, &SignedFlag);

  etimer_set(&temp_timer,CLOCK_SECOND * 3);

  while(1) {
    PROCESS_YIELD();
    if(ev == sensors_event && data == &button_sensor) {
         button_status=button_sensor.value(5);
         if(button_status&BUTTON_MASK)  //auto or manual
	     button_count[0]=1-button_count[0];	

         if(button_status&BUTTON2_MASK){  // manual to switch
	     button_count[1]=1-button_count[1];	
             if (button_count[0]==1){
                if(button_count[1]==0)
                    PWM_change(1, 0);
                 else
                    PWM_change(1, 255);
              }   
          }
 
         if(button_status&BUTTON_IR_MASK){
             if (button_count[0]==0){
                  button_count[2]=1;
                  button_count[3]=0;
                  button_count[4]++; 
              }
          }
  
     }
 

    if(etimer_expired(&temp_timer))       
    {
       if (button_count[2]==0){
           button_count[3]++;
           // no one for 30s
           if(button_count[3]>10 && button_count[4]!=0){
              sprintf(buf,"F2,0");
              simple_udp_send(server_conn, buf, strlen(buf)+1);      
              button_count[4]=0; 
          //button_count[4]!=0 means the F2,0 haven't been sent
            }
       }    
      button_count[2]=0;

      if(button_count[4]==1){ //make sure F2,1 only be sent once
              sprintf(buf,"F2,1");
              simple_udp_send(server_conn, buf, strlen(buf)+1);      
       }

       rv = sensor->value(ADC_SENSOR_TYPE_VDD);
       if(rv != -1) {
          voltage = rv * 3.75 / 2047;
          current= TM7709_read_count();
          power=voltage*current;
        
          dec = power;
          frac = power - dec;

          sprintf(buf,"A2,%d.%02u", dec, (unsigned int)(frac*100));
          simple_udp_send(server_conn, buf, strlen(buf)+1);      
       } 

      
      ReadTemp(&TempInt, &TempDot, &SignedFlag);
      printf("DS18b20:%c%d.%d\n",SignedFlag?'-':' ',TempInt,TempDot);
      if(TempInt>30){
         sprintf(buf_switch,"C4,2");
         PRINTF("DATA send to %d %s\n",
         switch_ipaddr.u8[sizeof(switch_ipaddr.u8) - 1],buf_switch);
         simple_udp_send(switch_conn, buf, strlen(buf)+1); 
      }

      etimer_reset(&temp_timer);
    }
  }
  PROCESS_END();
}

static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{

     memset(buf_data, 0, MAX_PAYLOAD_LEN);
  
     memcpy(buf_data, data, datalen);
     temp_center = atoi(buf_data);

    if(temp_center>32){
        PWM_change(0, LARGE_POWER);
        sprintf(buf,"B2,2");
     }
     else if(temp_center>27){
        PWM_change(0, SMALL_POWER);
        sprintf(buf,"B2,1");
     }
     else if(temp_center<27){
        PWM_change(0, 0);
        sprintf(buf,"B2,0");
     } 
     
     simple_udp_send(server_conn, buf, strlen(buf)+1);      
  
     return;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();

  temp_center=0;

  PRINTF("UDP client process started\n");

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 3);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);

  uip_ip6addr(&server_ipaddr,0xaaaa,0,0,0,0,0,0,1);
  /* new connection with remote host */
  if(simple_udp_register(server_conn, 3001,&server_ipaddr, 3000, receiver)==1)
     PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

  PRINTF("Connection with ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
  UIP_HTONS(server_conn->lport), UIP_HTONS(server_conn->rport));

  //!!!!!!!!!!!must be the local address from mac address
  //please test the address of switch mote and replace ?
  uip_ip6addr(&switch_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 8);
   if(simple_udp_register(switch_conn, 3002,NULL, 3000, receiver)==1)
     PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);
  

  while(1) {
    PROCESS_YIELD();
     PROCESS_WAIT_EVENT();
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
