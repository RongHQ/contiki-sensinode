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
#include "sys/ctimer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#include "dev/leds.h"
#include "dev/adc-sensor.h"
#include "dev/button-sensor.h"
#include "dev/DS18b20.h"
#include "dev/TM7709.h"
#include "dev/HX711.h"
#include "dev/timer4.h"
#include "dev/button-sensor.h"
#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		20
#define PRICE_DO                1

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

static int button_count[2]={0,0};
static int emer;
static char buf[MAX_PAYLOAD_LEN];
static char buf_data[MAX_PAYLOAD_LEN];
static char buf_switch[MAX_PAYLOAD_LEN];

static clock_time_t finish_time;
static int wash_finish;
static int price;
static int turn; 
static struct ctimer t,wash_t,t_wait;

/* Our destinations and udp conns. One link-local and one global */
#define LOCAL_CONN_PORT 3002
#define UDP_CLIENT_PORT 3001
#define UDP_SERVER_PORT 3000
static struct uip_udp_conn *server_conn;
static uip_ipaddr_t server_ipaddr, switch_ipaddr;



/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(wash_process, "washing process");
AUTOSTART_PROCESSES(&udp_client_process, &wash_process);
/*---------------------------------------------------------------------------*/
static void
send_packet(void)
{
  PRINTF("DATA send to %d 'Hello'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);
  uip_udp_packet_sendto(server_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
  return;
}


static void 
wash_off(void *ptr){
 
    PWM_change(1,0); //switch off
    button_count[1]=0; // means the washing button enable when manual
    wash_finish=1; //means finish washing
    return;
}

static void 
washing(void *ptr){
   
    // if already washing, watch again after 5 seconds
    // this condition appears when auto washing not finish 
    // and the mauanl washing task coming
    if(wash_finish==0){
       if(finish_time>5)
           finish_time-=5;
       ctimer_set(&t_wait, 5*CLOCK_SECOND,washing,ptr); 
    }
    // else start wash 30s, wash_finish=0 means washing,1 means finish washing
    else{
        wash_finish=0;
        PWM_change(1,255);
        ctimer_set(&wash_t, 30*CLOCK_SECOND,wash_off,NULL);
    }
   return;
}

static void
watch_queue(void *ptr)
{
   // loop until turn ==0
   sprintf(buf,"T=%d, f=%d", turn, finish_time);

   if(turn>0){
       ctimer_set(&t, finish_time*CLOCK_SECOND,watch_queue,ptr);
       turn--;
       finish_time=255;
   }

   // >4 min do nothing
   else if (finish_time>240){
       ctimer_set(&t, (finish_time-240)*CLOCK_SECOND,watch_queue,ptr);
       finish_time=240;
   }
   // >1 min look for the price
   else if (finish_time>60){
         if(price>PRICE_DO){
              ctimer_set(&t, 5*CLOCK_SECOND,watch_queue,ptr);
              finish_time-=5;
         }
         else{
        	 washing(NULL);
         }

   }
   // < 1 min look for emer and price 
   else{
         if (emer)
              washing(NULL);
         else{
              if(price>PRICE_DO){
                  if(finish_time>=5){ //stop before overflow
                     ctimer_set(&t, 5*CLOCK_SECOND,watch_queue,ptr);
                     finish_time-=5;
                  }      
              }
              else{
                  washing(NULL);
              }
         }   
    }
   return;
}




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
  
  etimer_set(&temp_timer,CLOCK_SECOND * 3);

  while(1) {
    PROCESS_YIELD();
     if(ev == sensors_event && data == &button_sensor) {
         button_status=button_sensor.value(5);
         if(button_status&BUTTON_MASK)  //auto or manual
	     button_count[0]=1-button_count[0];	

         if(button_status&BUTTON2_MASK){  // B button
             if (button_count[0]==1){
                  button_count[1]++; // ingore the press except the first
                  if(button_count[1]==1)
                      washing(NULL);        
              } 
              else{ // auto , press B to stop watch queue
                  button_count[1]=1-button_count[1];  
                  if(button_count[1]==0)
                       ctimer_stop(&t);
              }
          }
   
     }


    if(etimer_expired(&temp_timer))       
    {
      etimer_reset(&temp_timer);
      if(buf[0]=='T'){
    	  send_packet();
    	  buf[0] = 0;
      }
      rv = sensor->value(ADC_SENSOR_TYPE_VDD);
      if(rv != -1) {
        voltage = rv * 3.75 / 2047;
        current= TM7709_read_count();
        power=voltage*current;
        
        dec = power;
        frac = power - dec;

        sprintf(buf,"A3,%d.%02u", dec, (unsigned int)(frac*100));
        send_packet();
      }   

      ReadTemp(&TempInt, &TempDot, &SignedFlag);
      printf("DS18b20:%c%d.%d\n",SignedFlag?'-':' ',TempInt,TempDot);
      if(TempInt>30){
          sprintf(buf_switch,"C4,3");
         PRINTF("DATA send to %d %s\n",
         switch_ipaddr.u8[sizeof(switch_ipaddr.u8) - 1],buf_switch);
         uip_udp_packet_sendto(server_conn, buf_switch, strlen(buf_switch),
                        &switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

      }
      leds_toggle(LEDS_RED);
    }
  }
  PROCESS_END();
}

static void
tcpip_handler(void)
{
  memset(buf_data, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
     memcpy(buf_data, uip_appdata, uip_datalen());
    if(buf_data[0]=='C'){
        if (buf_data[1]=='0'){
        	emer=0;
        	PWM_change(0,255);
        }

        else{
        	emer=1;
        	PWM_change(0,0);
        }
        // if time>256, divide by 255 to prevent overflow

        turn=atoi(& buf_data[3]);
        finish_time= turn%255; // when to finish
        turn= (int) turn/255;
        watch_queue(NULL); // enter watch queue..
    }
    else if(buf_data[0] == 'B'){
        price = atoi(&buf_data[1]);  
    }
    else if(buf_data[0] == 'E'){
    	uip_ipaddr_copy(&switch_ipaddr, &UIP_IP_BUF->srcipaddr);
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

  price=0;  
  finish_time=0;
  wash_finish=1;

  PRINTF("UDP client process started\n");

  //uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 4);
  //uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);  //define in uip.h
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr); //define in uip_ds6.c convert mac to ipv6
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  uip_ip6addr(&server_ipaddr,0xaaaa,0,0,0,0,0,0,1);
  /* new connection with remote host */
  server_conn = udp_new(&server_ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL);
  if(!server_conn) {
    PRINTF("udp_new connenction error.\n");
  }
  udp_bind(server_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Connection with ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
  UIP_HTONS(server_conn->lport), UIP_HTONS(server_conn->rport));

 //!!!!!
  //uip_ip6addr(&switch_ipaddr,0xaaaa,0,0,0,0,0,0,5);// must be local address
  //switch_conn = udp_new(&switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL);
  //if(!switch_conn) {
    //PRINTF("switch connenction error.\n");
  //}
  //udp_bind(switch_conn, UIP_HTONS(LOCAL_CONN_PORT));
  

  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) 
      tcpip_handler();
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
