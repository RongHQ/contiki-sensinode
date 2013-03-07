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
#include "dev/TM7709.h"
#include "port.h"

#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define MAX_PAYLOAD_LEN		20



#define relay1 P1_0
#define relay2 P1_1
#define relay3 P1_2
#define relay4 P1_3
#define relay5 P2_0

static char buf[MAX_PAYLOAD_LEN];
static char buf_data[MAX_PAYLOAD_LEN];
static char buf_send[MAX_PAYLOAD_LEN];
static int warn_count;

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
  return;
}

PROCESS_THREAD(temp_process, ev, data)
{
  static struct etimer temp_timer;
  static struct sensors_sensor * sensor;
  static int light_sense;
  static int rv;
  static float voltage = 0;
  static int current=0;
  static float power;
  static int dec;
  static float frac;
  uip_ipaddr_t broad_addr;
  int button_status;


  PROCESS_BEGIN();

  PROCESS_PAUSE();


//initialize

  //leds_init();

  ADC_SENSOR_ACTIVATE();
  sensor = sensors_find(ADC_SENSOR);
   if (sensor) {
      printf("Sensor OK!\n");
      //leds_on(LEDS_GREEN);
      sensor->configure(SENSORS_HW_INIT,1);
  }

  TM7709_init();

  PORT_FUNC_GPIO_X(1,0);
    PORT_DIR_OUTPUT_X(1,0);

    PORT_FUNC_GPIO_X(1,1);
    PORT_DIR_OUTPUT_X(1,1);

    PORT_FUNC_GPIO_X(2,0);
    PORT_DIR_OUTPUT_X(2,0);

    PORT_FUNC_GPIO_X(2,1);
    PORT_DIR_OUTPUT_X(1,2);

    PORT_FUNC_GPIO_X(2,2);
    PORT_DIR_OUTPUT_X(1,3);


  // read the sensor every 3 seconds
  etimer_set(&temp_timer,CLOCK_SECOND * 3);
  uip_create_linklocal_allnodes_mcast(&broad_addr);

  while(1) {
    PROCESS_YIELD();

    if(etimer_expired(&temp_timer))
    {
      etimer_reset(&temp_timer);
      //read the power value
      rv = sensor->value(ADC_SENSOR_TYPE_VDD);
      if(rv != -1) {
         voltage = rv * 3.75 / 2047;
         current= TM7709_read_count();
         power=voltage*current;

         dec = power;
         frac = power - dec;

         // two decimal places
         sprintf(buf,"A4,%d.%02u", dec, (unsigned int)(frac*100));
         send_packet();
       }
      sprintf(buf,"E");
      send_packet();

      sprintf(buf,"E");
      uip_udp_packet_sendto(switch_conn, buf, strlen(buf),
                            &broad_addr, UIP_HTONS(LOCAL_CONN_PORT));

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
        // the first warn
        if(warn_count==0)
            memcpy(buf_send, buf_data, MAX_PAYLOAD_LEN); 
        // the second warn, send this and last warning message
        else if(warn_count==1){
            memcpy(buf, buf_send, MAX_PAYLOAD_LEN);
            send_packet();
            memcpy(buf, uip_appdata, uip_datalen());
            send_packet();
        }
        // >second, send warning meesage
        else if(warn_count>1){
            memcpy(buf, uip_appdata, uip_datalen());
            send_packet();
        }
        warn_count++;
     }
     // switch message from center
     else if(buf_data[0]=='F'){
         warn_count=0;
         if(buf_data[1]=='1'){
               relay1=1;
               relay2=1;
               relay3=0;
               relay4=0;
               relay5=0;
          }
          else if(buf_data[1]=='2'){
               relay1=1;
               relay2=0;
               relay3=0;
               relay4=1;
               relay5=0;
          }
          else if(buf_data[1]=='3'){
        	   relay1=0;
        	   relay2=0;
        	   relay3=0;
        	   relay4=0;
			   relay5=1;
          }
          else if(buf_data[1]=='4'){
        	   relay1=0;
        	   relay2=0;
               relay3=1;
               relay4=0;
               relay5=1;
           }
     }
  }

  return;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  //static struct etimer et;
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  warn_count=0; 
  memset(buf_send, 0, MAX_PAYLOAD_LEN);

  PRINTF("UDP client process started\n");

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
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

  switch_conn = udp_new(NULL, UIP_HTONS(LOCAL_CONN_PORT), NULL);
  if(!switch_conn) {
    PRINTF("switch connenction error.\n");
  }
  udp_bind(switch_conn, UIP_HTONS(UDP_SERVER_PORT));

  //etimer_set(&et,CLOCK_SECOND);
  
  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) 
      tcpip_handler();
    

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
