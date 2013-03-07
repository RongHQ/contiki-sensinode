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
#include "sys/rtimer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#include "dev/leds.h"
#include "dev/DS18b20.h"
#include "dev/HX711.h"
#include "dev/timer4.h"
#include "dev/button-sensor.h"
#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		40
#define PRICE_DO                1

static int button_count;
static int emer;
static char buf[MAX_PAYLOAD_LEN];
static char data[MAX_PAYLOAD_LEN];
static clock_time_t finish_time;
static int price;
static struct ctimer t;

/* Our destinations and udp conns. One link-local and one global */
#define LOCAL_CONN_PORT 3002
#define UDP_CLIENT_PORT 3001
#define UDP_SERVER_PORT 3000
static struct uip_udp_conn *server_conn, *switch_conn;
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
         switch_ipaddr.u8[sizeof(switch_ipaddr.u8) - 1]);
  uip_udp_packet_sendto(switch_conn, buf, strlen(buf),
                        &switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
  return;
}

static void 
washing(void){
    leds_on(LEDS_GREEN);
}

static void
watch_queue (void *ptr)
{
    if ((clock_time_t )RTIMER_NOW()<finish_time-60*CLOCK_SECOND){ 
         if(price>PRICE_DO)
              ctimer_set(&t, 3*CLOCK_SECOND,watch_queue,ptr);
         else 
              washing();
     }
     else if (emer)
          washing();
     else
         ctimer_set(&t, 3*CLOCK_SECOND,watch_queue,ptr);     
       
}

PROCESS_THREAD(wash_process, ev, data)
{
  static struct etimer temp_timer;

   uchar TempInt,TempDot,SignedFlag;

  PROCESS_BEGIN();
  
  PROCESS_PAUSE();

  etimer_set(&temp_timer,CLOCK_SECOND * 5);

  if ((clock_time_t )RTIMER_NOW()<finish_time-240*CLOCK_SECOND) 
        ctimer_set(&t, finish_time-240*CLOCK_SECOND,watch_queue, NULL);
  else if (RTIMER_CLOCK_LT(RTIMER_NOW()+RTIMER_ARCH_SECOND,finish_time)) 
          ctimer_set(&t, finish_time-240*CLOCK_SECOND,watch_queue, NULL);
  else 
      washing();

  while(1) {
    PROCESS_YIELD();
    if(ev == sensors_event && data == &button_sensor) {
      button_count=1-button_count;
    }
    if (!button_count)
       ctimer_stop(&t);

    if(etimer_expired(&temp_timer))       
    {
      ReadTemp(&TempInt, &TempDot, &SignedFlag);
      printf("DS18b20:%c%d.%d\n",SignedFlag?'-':' ',TempInt,TempDot);
      if(TempInt>30){
         sprintf(buf, "warning!");
         send_packet();
      }
      etimer_reset(&temp_timer);
    }
  }
  PROCESS_END();
}

static void
tcpip_handler(void)
{ 
  if(uip_newdata()) {
     memcpy(data, uip_appdata, sizeof(uip_appdata));
    if(data[0]=='1'){
        emer=atoi(data[1]);
        finish_time= atoi(& data[2]);
    }
    else
        price = atoi(&data[1]);  
  }

}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  

  PROCESS_BEGIN();

  price=0;  
  finish_time=250*CLOCK_SECOND; 

  PRINTF("UDP client process started\n");

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
  uip_ip6addr(&switch_ipaddr,?);// must be local address
  switch_conn = udp_new(&switch_ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL);
  if(!switch_conn) {
    PRINTF("switch connenction error.\n");
  }
  udp_bind(switch_conn, UIP_HTONS(LOCAL_CONN_PORT));
  

  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) 
      tcpip_handler();
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
