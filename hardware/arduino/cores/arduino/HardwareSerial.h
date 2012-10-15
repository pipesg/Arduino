/*
  HardwareSerial.h - Hardware serial library for Wiring
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

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"

typedef enum {
#if defined(UBRRH) || defined(UBRR0H)
    HARDWARE_SERIAL = 0,
#endif
#if defined(UBRR1H)
    HARDWARE_SERIAL1 = 1,
#endif
#if defined(UBRR2H)
    HARDWARE_SERIAL2 = 2,
#endif
#if defined(UBRR3H)
    HARDWARE_SERIAL3 = 3,
#endif
} SerialPort;

#if (RAMEND < 1000)
#define DEFAULT_SERIAL_BUFFER_SIZE 16
#else
#define DEFAULT_SERIAL_BUFFER_SIZE 64
#endif

typedef struct
{
    unsigned char * buffer;
    volatile unsigned int buffer_size;
    volatile unsigned int head;
    volatile unsigned int tail;
} ring_buffer;

#if defined(UBRRH) || defined(UBRR0H)
void set_rx_buffer(ring_buffer *);
void set_tx_buffer(ring_buffer *);
#endif
#if defined(UBRR1H)
void set_rx_buffer1(ring_buffer *);
void set_tx_buffer1(ring_buffer *);
#endif
#if defined(UBRR2H)
void set_rx_buffer2(ring_buffer *);
void set_tx_buffer2(ring_buffer *);
#endif
#if defined(UBRR3H)
void set_rx_buffer3(ring_buffer *);
void set_tx_buffer3(ring_buffer *);
#endif

template<SerialPort S, 
    int RX_SERIAL_BUFFER_SIZE = DEFAULT_SERIAL_BUFFER_SIZE, 
    int TX_SERIAL_BUFFER_SIZE = DEFAULT_SERIAL_BUFFER_SIZE> 
    class HardwareSerial : public Stream
    {
    private:

    unsigned char _rx_buffer_buf[RX_SERIAL_BUFFER_SIZE];
    unsigned char _tx_buffer_buf[TX_SERIAL_BUFFER_SIZE];

    ring_buffer _rx_buffer;
    ring_buffer _tx_buffer;
    volatile uint8_t *_ubrrh;
    volatile uint8_t *_ubrrl;
    volatile uint8_t *_ucsra;
    volatile uint8_t *_ucsrb;
    volatile uint8_t *_udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udrie;
    uint8_t _u2x;
    public:

    HardwareSerial();
    void begin(unsigned long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool();
    };


template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::HardwareSerial()
{
    _rx_buffer.buffer = _rx_buffer_buf;
    _rx_buffer.buffer_size = RX_SERIAL_BUFFER_SIZE;
    _tx_buffer.buffer = _tx_buffer_buf;
    _tx_buffer.buffer_size = TX_SERIAL_BUFFER_SIZE;
    switch (serialPort) {
#if defined(UBRRH) || defined(UBRR0H)
    case HARDWARE_SERIAL:
#if defined(UBRRH) && defined(UBRRL)
        _ubrrh = &UBRRH;
        _ubrrl = &UBRRL;
        _ucsra = &UCSRA;
        _ucsrb = &UCSRB;
        _udr = &UDR;
        _rxen = RXEN;
        _txen = TXEN;
        _rxcie = RXCIE;
        _udrie = UDRIE;
        _u2x = U2X;
#elif defined(UBRR0H) && defined(UBRR0L)
        _ubrrh = &UBRR0H;
        _ubrrl = &UBRR0L;
        _ucsra = &UCSR0A;
        _ucsrb = &UCSR0B;
        _udr = &UDR0;
        _rxen = RXEN0;
        _txen = TXEN0;
        _rxcie = RXCIE0;
        _udrie = UDRIE0;
        _u2x = U2X0;
#endif
        set_rx_buffer(&_rx_buffer);
        set_tx_buffer(&_tx_buffer);
        break;
#endif
#if defined(UBRR1H)
    case HARDWARE_SERIAL1:
        _ubrrh = &UBRR1H;
        _ubrrl = &UBRR1L;
        _ucsra = &UCSR1A;
        _ucsrb = &UCSR1B;
        _udr = &UDR1;
        _rxen = RXEN1;
        _txen = TXEN1;
        _rxcie = RXCIE1;
        _udrie = UDRIE1;
        _u2x = U2X1;
        set_rx_buffer1(&_rx_buffer);
        set_tx_buffer1(&_tx_buffer);
        break;
#endif
#if defined(UBRR2H)
    case HARDWARE_SERIAL2:
        _ubrrh = &UBRR2H;
        _ubrrl = &UBRR2L;
        _ucsra = &UCSR2A;
        _ucsrb = &UCSR2B;
        _udr = &UDR2;
        _rxen = RXEN2;
        _txen = TXEN2;
        _rxcie = RXCIE2;
        _udrie = UDRIE2;
        _u2x = U2X2;
        set_rx_buffer2(&_rx_buffer);
        set_tx_buffer2(&_tx_buffer);
        break;
#endif
#if defined(UBRR3H)
    case HARDWARE_SERIAL3:
        _ubrrh = &UBRR3H;
        _ubrrl = &UBRR3L;
        _ucsra = &UCSR3A;
        _ucsrb = &UCSR3B;
        _udr = &UDR3;
        _rxen = RXEN3;
        _txen = TXEN3;
        _rxcie = RXCIE3;
        _udrie = UDRIE3;
        _u2x = U2X3;
        set_rx_buffer3(&_rx_buffer);
        set_tx_buffer3(&_tx_buffer);
        break;
#endif
    }
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
void
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::begin(unsigned long baud)
{
    uint16_t baud_setting;
    bool use_u2x = true;

#if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600) {
        use_u2x = false;
    }
#endif

 try_again:
  
    if (use_u2x) {
        *_ucsra = 1 << _u2x;
        baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
        *_ucsra = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }
  
    if ((baud_setting > 4095) && use_u2x)
        {
            use_u2x = false;
            goto try_again;
        }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    *_ubrrh = baud_setting >> 8;
    *_ubrrl = baud_setting;

    *_ucsrb |= _BV(_rxen);
    *_ucsrb |= _BV(_txen);
    *_ucsrb |= _BV(_rxcie);
    *_ucsrb &= ~(_BV(_udrie));
}
template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
void
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::end()
{
  // wait for transmission of outgoing data
  while (_tx_buffer.head != _tx_buffer.tail)
    ;

  *_ucsrb &= ~(_BV(_rxen));
  *_ucsrb &= ~(_BV(_txen));
  *_ucsrb &= ~(_BV(_rxcie));  
  *_ucsrb &= ~(_BV(_udrie));
  
  // clear any received data
  _rx_buffer.head = _rx_buffer.tail;
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
int
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::available(void)
{
  return (unsigned int)(RX_SERIAL_BUFFER_SIZE + _rx_buffer.head - _rx_buffer.tail) % RX_SERIAL_BUFFER_SIZE;
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
int
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::peek(void)
{
  if (_rx_buffer.head == _rx_buffer.tail) {
    return -1;
  } else {
    return _rx_buffer.buffer[_rx_buffer.tail];
  }
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
int
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer.head == _rx_buffer.tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer.buffer[_rx_buffer.tail];
    _rx_buffer.tail = (unsigned int)(_rx_buffer.tail + 1) % TX_SERIAL_BUFFER_SIZE;
    return c;
  }
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
void
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::flush()
{
  while (_tx_buffer.head != _tx_buffer.tail)
    ;
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
size_t
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::write(uint8_t c)
{
  unsigned int i = (_tx_buffer.head + 1) % RX_SERIAL_BUFFER_SIZE;
	
  // If the output buffer is full, there's nothing for it other than to 
  // wait for the interrupt handler to empty it a bit
  // ???: return 0 here instead?
  while (i == _tx_buffer.tail)
    ;
	
  _tx_buffer.buffer[_tx_buffer.head] = c;
  _tx_buffer.head = i;
	
  *_ucsrb |= _BV(_udrie);
  
  return 1;
}

template <SerialPort serialPort, int RX_SERIAL_BUFFER_SIZE, int TX_SERIAL_BUFFER_SIZE> 
HardwareSerial<serialPort, RX_SERIAL_BUFFER_SIZE, TX_SERIAL_BUFFER_SIZE>
::operator bool() {
	return true;
}

/* JD temporary disabled
   extern void serialEventRun(void) __attribute__((weak));
*/

#endif
