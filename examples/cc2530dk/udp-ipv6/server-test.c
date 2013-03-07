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

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/uart0.h"
#include "dev/serial-line.h"
#include "debug.h"

#define MAX_PAYLOAD_LEN 20

static struct uip_udp_conn *server_conn;



#define SERVER_REPLY          1
#define UDP_CLIENT_PORT 3001
#define UDP_SERVER_PORT 3000

/* Should we act as RPL root? */
#define SERVER_RPL_ROOT       1

#if SERVER_RPL_ROOT
static uip_ipaddr_t ipaddr;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/

static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
 
    printf("%s",data);
    // printf the message recevied

}


static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state
        == ADDR_PREFERRED)) {
      PRINTF("  ");
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
#if SERVER_RPL_ROOT
void
create_dag()
{
  rpl_dag_t *dag;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);  //define in uip.h
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);

 // print_local_addresses();

   // dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    dag = rpl_set_root((uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    
  
   // PRINTF("Created a new RPL dag with ID: ");
    //PRINT6ADDR(&dag->dag_id);
    //PRINTF("\n");
  
}
#endif /* SERVER_RPL_ROOT */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{

  PROCESS_BEGIN();
  putstring("Starting UDP server\n");

#if SERVER_RPL_ROOT
  create_dag();
#endif

if(simple_udp_register(server_conn, 3000,NULL, 3001, receiver)==1)
  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);


  while(1) {
    PROCESS_YIELD();
    PROCESS_WAIT_EVENT();
    if(ev == serial_line_event_message){
       static char line_data[MAX_PAYLOAD_LEN];
       static uip_ipaddr_t mote_ipaddr;

       memset(buf, 0, MAX_PAYLOAD_LEN); 
       memset(line_data, 0, MAX_PAYLOAD_LEN);
       memcpy(line_data, data, strlen(data));       
       char buf[MAX_PAYLOAD_LEN];
       //air
       if(line_data[0]=='A'){ 
           memcpy(buf, &line_data[1], strlen(line_data)-1);
           uip_ip6addr(&mote_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 3);
           simple_udp_sendto(server_conn, buf, strlen(buf)+1, &mote_ipaddr);
        }
        //washing
        if(line_data[0]=='B' || line_data[0]=='C'){ 
           memcpy(buf, line_data, strlen(line_data));
           uip_ip6addr(&mote_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 4);
           simple_udp_sendto(server_conn, buf, strlen(buf)+1, &mote_ipaddr);
        }
        //switch
        else if(line_data[0]=='F'){ 
           memcpy(buf, line_data, sizeof(line_data));
           uip_ip6addr(&mote_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 5);
           simple_udp_sendto(server_conn, buf, strlen(buf)+1, &mote_ipaddr);
        }
 
       
    }   
  }    

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
