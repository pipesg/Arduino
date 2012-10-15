/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "wiring_private.h"

// this next line disables the entire HardwareSerial.cpp, 
// this is so I can support Attiny series and any other chip without a uart
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#include "HardwareSerial.h"

#if defined(USBCON)
static ring_buffer * rx_buffer = 0;
void set_rx_buffer(ring_buffer * buffer) { rx_buffer = buffer; }
static ring_buffer * tx_buffer = 0;
void set_tx_buffer(ring_buffer * buffer) { tx_buffer = buffer; }
#endif
#if defined(UBRRH) || defined(UBRR0H)
static ring_buffer * rx_buffer  =  0;
void set_rx_buffer(ring_buffer * buffer) { rx_buffer = buffer; }
static ring_buffer * tx_buffer  =  0;
void set_tx_buffer(ring_buffer * buffer) { tx_buffer = buffer; }
#endif
#if defined(UBRR1H)
static ring_buffer * rx_buffer1  = 0;
void set_rx_buffer1(ring_buffer * buffer) { rx_buffer1 = buffer; }
static ring_buffer * tx_buffer1  = 0; 
void set_tx_buffer1(ring_buffer * buffer) { tx_buffer1 = buffer; }
#endif
#if defined(UBRR2H)
static ring_buffer * rx_buffer2  = 0;
void set_rx_buffer2(ring_buffer * buffer) { rx_buffer2 = buffer; }
static ring_buffer * tx_buffer2  = 0;
void set_tx_buffer2(ring_buffer * buffer) { tx_buffer2 = buffer; }
#endif
#if defined(UBRR3H)
static ring_buffer * rx_buffer3  = 0; 
void set_rx_buffer3(ring_buffer * buffer) { rx_buffer3 = buffer; }
static ring_buffer * tx_buffer3  = 0;
void set_tx_buffer3(ring_buffer * buffer) { tx_buffer3 = buffer; }
#endif


inline void store_char(unsigned char c, ring_buffer *buffer)
{
  unsigned int i = (unsigned int)(buffer->head + 1) % (buffer->buffer_size);

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(USART_RX_vect) && !defined(SIG_USART0_RECV) && \
    !defined(SIG_UART0_RECV) && !defined(USART0_RX_vect) && \
	!defined(SIG_UART_RECV)
  #error "Don't know what the Data Received vector is called for the first UART"
#else
  void serialEvent() __attribute__((weak));
  void serialEvent() {}
  #define serialEvent_implemented
#if defined(USART_RX_vect)
  SIGNAL(USART_RX_vect)
#elif defined(SIG_USART0_RECV)
  SIGNAL(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
  SIGNAL(SIG_UART0_RECV)
#elif defined(USART0_RX_vect)
  SIGNAL(USART0_RX_vect)
#elif defined(SIG_UART_RECV)
  SIGNAL(SIG_UART_RECV)
#endif
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;
  #elif defined(UDR)
    unsigned char c  =  UDR;
  #else
    #error UDR not defined
  #endif
    store_char(c, rx_buffer);
  }
#endif
#endif

#if defined(USART1_RX_vect)
  void serialEvent1() __attribute__((weak));
  void serialEvent1() {}
  #define serialEvent1_implemented
  SIGNAL(USART1_RX_vect)
  {
    unsigned char c = UDR1;
    store_char(c, rx_buffer1);
  }
#elif defined(SIG_USART1_RECV)
  #error SIG_USART1_RECV
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
  void serialEvent2() __attribute__((weak));
  void serialEvent2() {}
  #define serialEvent2_implemented
  SIGNAL(USART2_RX_vect)
  {
    unsigned char c = UDR2;
    store_char(c, rx_buffer2);
  }
#elif defined(SIG_USART2_RECV)
  #error SIG_USART2_RECV
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
  void serialEvent3() __attribute__((weak));
  void serialEvent3() {}
  #define serialEvent3_implemented
  SIGNAL(USART3_RX_vect)
  {
    unsigned char c = UDR3;
    store_char(c, rx_buffer3);
  }
#elif defined(SIG_USART3_RECV)
  #error SIG_USART3_RECV
#endif

/* JD temporary disabled
void serialEventRun(void)
{
#ifdef serialEvent_implemented
  if (Serial.available()) serialEvent();
#endif
#ifdef serialEvent1_implemented
  if (Serial1.available()) serialEvent1();
#endif
#ifdef serialEvent2_implemented
  if (Serial2.available()) serialEvent2();
#endif
#ifdef serialEvent3_implemented
  if (Serial3.available()) serialEvent3();
#endif
}
*/


#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
  #error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
    if (tx_buffer == 0) return;
    if (tx_buffer->head == tx_buffer->tail) {
	// Buffer empty, so disable interrupts
#if defined(UCSR0B)
        cbi(UCSR0B, UDRIE0);
#else
        cbi(UCSRB, UDRIE);
#endif
    }
    else {
        // There is more data in the output buffer. Send the next byte
        unsigned char c = tx_buffer->buffer[tx_buffer->tail];
        tx_buffer->tail = (tx_buffer->tail + 1) % (tx_buffer->buffer_size);
	
#if defined(UDR0)
        UDR0 = c;
#elif defined(UDR)
        UDR = c;
#else
#error UDR not defined
#endif
    }
}
#endif
#endif

#ifdef USART1_UDRE_vect
ISR(USART1_UDRE_vect)
{
    if (tx_buffer1 == 0) return;
    if (tx_buffer1->head == tx_buffer1->tail) {
	// Buffer empty, so disable interrupts
        cbi(UCSR1B, UDRIE1);
    }
    else {
        // There is more data in the output buffer. Send the next byte
        unsigned char c = tx_buffer1->buffer[tx_buffer1->tail];
        tx_buffer1->tail = (tx_buffer1->tail + 1) % (tx_buffer1->buffer_size);
	
        UDR1 = c;
    }
}
#endif

#ifdef USART2_UDRE_vect
ISR(USART2_UDRE_vect)
{
    if (tx_buffer2 == 0) return;
    if (tx_buffer2->head == tx_buffer2->tail) {
	// Buffer empty, so disable interrupts
        cbi(UCSR2B, UDRIE2);
    }
    else {
        // There is more data in the output buffer. Send the next byte
        unsigned char c = tx_buffer2->buffer[tx_buffer2->tail];
        tx_buffer2->tail = (tx_buffer2->tail + 1) % (tx_buffer2->buffer_size);
	
        UDR2 = c;
    }
}
#endif

#ifdef USART3_UDRE_vect
ISR(USART3_UDRE_vect)
{
    if (tx_buffer3 == 0) return;
    if (tx_buffer3->head == tx_buffer3->tail) {
	// Buffer empty, so disable interrupts
        cbi(UCSR3B, UDRIE3);
    }
    else {
        // There is more data in the output buffer. Send the next byte
        unsigned char c = tx_buffer3->buffer[tx_buffer->tail];
    tx_buffer3->tail = (tx_buffer3->tail + 1) % (tx_buffer3->buffer_size):
    
    UDR3 = c;
    }
}
#endif

#endif // whole file

