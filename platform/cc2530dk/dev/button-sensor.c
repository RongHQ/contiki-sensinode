/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
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
 */

/*
 * This file contains ISRs: Keep it in the HOME bank.
 */
#include "dev/port.h"
#include "dev/button-sensor.h"
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
static __data struct timer debouncetimer[3];
static __data unsigned char button_status=0;
/*---------------------------------------------------------------------------*/
static
int value(int type)
{
  unsigned char temp;
  temp = button_status;
  button_status = 0;
  return (int)temp;
}
/*---------------------------------------------------------------------------*/
static
int status(int type)
{
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return ((BUTTON_IRQ_ENABLED())||
  		  (BUTTON2_IRQ_ENABLED())<<1||
  		  (BUTTON_IR_IRQ_ENABLED())<<2);
    }
  return 0;
}
/*---------------------------------------------------------------------------*/
static
int configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    P0INP |= 2; /* Tri-state */
    BUTTON_IRQ_ON_PRESS();
    //BUTTON2_IRQ_ON_PRESS();
    //BUTTON_IR_IRQ_ON_PRESS();

    BUTTON_FUNC_GPIO();
    BUTTON2_FUNC_GPIO();
    BUTTON_IR_FUNC_GPIO();

    BUTTON_DIR_INPUT();
    BUTTON2_DIR_INPUT();
    BUTTON_IR_DIR_INPUT();
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      if(!BUTTON_IRQ_ENABLED()) {
        timer_set(&debouncetimer[0], 0);
        BUTTON_IRQ_FLAG_OFF();
        BUTTON_IRQ_ENABLE();
      }
      if(!BUTTON2_IRQ_ENABLED()) {
        timer_set(&debouncetimer[1], 0);
        BUTTON2_IRQ_FLAG_OFF();
        BUTTON2_IRQ_ENABLE();
      }
      if(!BUTTON_IR_IRQ_ENABLED()) {
        timer_set(&debouncetimer[2], 0);
        BUTTON_IR_IRQ_FLAG_OFF();
        BUTTON_IR_IRQ_ENABLE();
      }
    } else {
      BUTTON_IRQ_DISABLE();
      BUTTON2_IRQ_DISABLE();
      BUTTON_IR_IRQ_DISABLE();
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
port_0_isr(void) __interrupt(P0INT_VECTOR)
{
  EA = 0;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /* This ISR is for the entire port. Check if the interrupt was caused by our
   * button's pin. */
  if(BUTTON_IRQ_CHECK()) {
    if(timer_expired(&debouncetimer[0])) {
      timer_set(&debouncetimer[0], CLOCK_SECOND / 4);
      button_status |= BUTTON_MASK;
      sensors_changed(&button_sensor);
    }
    BUTTON_IRQ_FLAG_OFF();
  }
  if(BUTTON2_IRQ_CHECK()) {
      if(timer_expired(&debouncetimer[1])) {
        timer_set(&debouncetimer[1], CLOCK_SECOND / 4);
        button_status |= BUTTON2_MASK;
        sensors_changed(&button_sensor);
      }
      BUTTON2_IRQ_FLAG_OFF();
    }
  if(BUTTON_IR_IRQ_CHECK()) {
      if(timer_expired(&debouncetimer[2])) {
        timer_set(&debouncetimer[2], CLOCK_SECOND / 4);
        button_status |= BUTTON_IR_MASK;
        sensors_changed(&button_sensor);
      }
      BUTTON_IR_IRQ_FLAG_OFF();
    }


  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  EA = 1;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR, value, configure, status);
#endif /* BUTTON_SENSOR_ON */
