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
#include "dev/dma.h"
#include "dev/cc2530-rf.h"
#include "contiki-lib.h"
#include "net/rime.h"
#include "cc253x.h"
//#include "contiki-net.h"
//#include "dev/leds-arch.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
PROCESS(ADC_DMA_process, "ADC DMA process");
AUTOSTART_PROCESSES(&hello_world_process, &ADC_DMA_process);
const rimeaddr_t sink_addr = {{26, 51}};
//extern process_event_t ADC_sensor_event;
static struct etimer timer1;
static uint8_t sample_count = 0;
static uint16_t max_tstamp = 0;
static int16_t sample_max = 0xFFFF;
int16_t ADC_buffer_ping[256], ADC_buffer_pang[256];
static int16_t *ADC_buffer;
static uint16_t tmp_ptr;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ADC_DMA_process, ev, data) {
  uint8_t i;
  PROCESS_BEGIN();

    tmp_ptr = (uint16_t) (&ADC_buffer_ping[0]);
    dma_conf[0].src_h = 0x70;
    dma_conf[0].src_l = 0xBA;
    dma_conf[0].dst_h = (uint8_t) (tmp_ptr >> 8);
    dma_conf[0].dst_l = (uint8_t) tmp_ptr;
    dma_conf[0].len_h = (uint8_t) (sizeof(ADC_buffer_ping) >> 9);
    dma_conf[0].len_l = (uint8_t) (sizeof(ADC_buffer_ping) >> 1);
    dma_conf[0].wtt = 0x80 | DMA_SINGLE | DMA_T_ADC_CH74;
    dma_conf[0].inc_prio = DMA_SRC_INC_NO | DMA_DST_INC_1 | DMA_PRIO_GUARANTEED;

    tmp_ptr = (uint16_t) (&ADC_buffer_pang[0]);
    dma_conf[1].src_h = 0x70;
    dma_conf[1].src_l = 0xBA;
    dma_conf[1].dst_h = (uint8_t) (tmp_ptr >> 8);
    dma_conf[1].dst_l = (uint8_t) tmp_ptr;
    dma_conf[1].len_h = (uint8_t) (sizeof(ADC_buffer_pang) >> 9);
    dma_conf[1].len_l = (uint8_t) (sizeof(ADC_buffer_pang) >> 1);
    dma_conf[1].wtt = 0x80 | DMA_SINGLE | DMA_T_ADC_CH74;
    dma_conf[1].inc_prio = DMA_SRC_INC_NO | DMA_DST_INC_1 | DMA_PRIO_GUARANTEED;

    dma_associate_process(&ADC_DMA_process, 0);
    dma_associate_process(&ADC_DMA_process, 1);
    //DMA_ARM(0);
    //printf("DMA config OK");

    while (1) {
      PROCESS_WAIT_EVENT();

      if (ev == PROCESS_EVENT_POLL) {

        if (DMAARM & 0x02) {
          ADC_buffer = ADC_buffer_ping;
          leds_on(LEDS_RED);
        }
        if (DMAARM & 0x01) {
          ADC_buffer = ADC_buffer_pang;
          leds_off(LEDS_RED);
        }

        i = 0;
        do {
          if (ADC_buffer[i] > sample_max) {
            sample_max = ADC_buffer[i];
            max_tstamp = (i | sample_count << 8);
          }
          i++;
        }
        while (i);
        sample_count++;
      }
    }

  PROCESS_END();
}

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from) {
  printf("unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
}

static const struct unicast_callbacks unicast_callbacks = { recv_uc };
static struct unicast_conn uc;

static void
abc_recv(struct abc_conn *c) {
  static int32_t t2stamp;
  //static char hexword[7] = {0,0,0,0,0,0,0};
  static uint8_t do_once = 1, DMA_sample_count_cap;
  //T2M0;

  DISABLE_INTERRUPTS();
  T2MSEL = 0x11;
  t2stamp = DMA_t2cap - SFD_T2cap;
  T2MSEL = 0x00;
  DMA_sample_count_cap = DMA_sample_count;
  ENABLE_INTERRUPTS();

  packetbuf_copyfrom((void *)&t2stamp, 4);
  packetbuf_appendfrom(&sample_count,1);
  packetbuf_appendfrom(&DMA_sample_count_cap,1);
  packetbuf_appendfrom(&max_tstamp,2);
  unicast_send(&uc, &sink_addr);
  max_tstamp = 0;
  sample_max = 0;

  if(do_once){
    do_once = 0;
    DMA_ARM(0);
  }
}
static const struct abc_callbacks abc_call = { abc_recv };
static struct abc_conn abc;

PROCESS_THREAD(hello_world_process, ev, data) {

  PROCESS_EXITHANDLER(unicast_close(&uc);abc_close(&abc);)

  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);
  abc_open(&abc, 128, &abc_call);

  printf("Hello World!\n");
  ADC_SENSOR_ACTIVATE();
  etimer_set(&timer1, CLOCK_SECOND);

  while (1) {
    PROCESS_WAIT_EVENT();

    if (ev == PROCESS_EVENT_TIMER && etimer_expired(&timer1)) {
      const rimeaddr_t addr = {{1, 0}};
      //printf("count%u",sample_count);
      //printf("max%d",sample_max);
      //printf("stamp%u\r\n",max_tstamp);
      //puthex(ADCCON1);
      //puthex(ADCCON2);
      //puthex(ADCL);
      //putchar('h');
      //putchar((unsigned char)(sample_count>>8));
      //putchar((unsigned char)sample_count);
      //sample_count = 0;
      //max_tstamp = 0;
      //sample_max = 0;
      leds_toggle(LEDS_GREEN);
      //leds_off(LEDS_GREEN);

      etimer_reset(&timer1);
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
