/**
 * \file
 *         RF driver ISRs
 * \author
 *         RONG HQ <ronghq@live.com>
 *
 *         DMA interrupt routines, must be stored in HOME bank
 */

#include "contiki.h"
#include "dev/cc2530-rf.h"
#include "cc253x.h"

volatile int32_t SFD_T2cap;

void RF_isr(void) __interrupt(RF_VECTOR){
  DISABLE_INTERRUPTS();
  T2MSEL = 0x11;
  SFD_T2cap = (T2MOVF0 | (T2MOVF1 << 8) | (T2MOVF2 << 16));
  T2MSEL = 0x00;

  RFIRQF0 = 0x00;
  S1CON = 0x00;
  ENABLE_INTERRUPTS();
}
