/*
 * mmtestmspi.ino
 */

#include <Arduino.h>

#define L0   78   // default for diesel engines
#define L1   72   // default for steam engines
#define L2   60   // default for rail cars
#define L3   24   // default for electric engines

// the MM protocol modulates polarity of the power supply on the tracks
// the different signals are being used which consist of two subpatterns:
//
//               |<----- 416µs ----->|
//               |<-208µs->|
//            -->| |<--26µs
//                _         _
// ternary 0:    | |       | |
//               | |_______| |_______
//                _______   _______
// ternary 1:    |       | |       |
//               |       |_|       |_
//                _______   _
// ternary 2:    |       | | |
//               |       |_| |_______
//
// The fourth possible state low/high is not used in the original protocol.
//
// This sketch uses the USART in MSPI mode to generate the trinary pattern. The
// 26µs peek is exactly 1/416 of the 16Mhz base clock.
// Each trinary state thus is being sent as two SPI data bytes:
// Trinary 0 is being sent as bytes 10000000, 10000000
// Trinary 1 is being sent as bytes 11111110, 11111110
// Trinary 2 is being sent as bytes 11111110, 10000000
// since the idle level of TX is high everything is sent inverted (as with uart)

// spi encoding of low-to-high or high-to-low event
#define U0  0b01111111
#define U1  0b00000001

void setup() {
  pinMode(17, OUTPUT);   // for debugging

  // init spi
  UCSR1C = (1<<UMSEL11)|(1<<UMSEL10); // set spi mode 0
  UCSR1B = (1<<TXEN1);                // enable transmitter only
  UBRR1 = 26*(F_CPU/2000000)-1;       // 26us bit length
}

void mm_tx(uint8_t *data, uint8_t len) {
  while(len--) {  
    while ( !(UCSR1A & _BV(UDRE1)) );   // wait for transmit buffer empty
    UDR1 = *data++;
  }
}

// encode address to ternary
void a2t(uint8_t b, uint8_t *d) {
  // ternary encoding:
  // 00 = 0
  // 11 = 1
  // 10 = third state
  // lsb is encoded into msb 
  uint8_t f = 27;

  d += 7;  // encode from end

  while(f) {
    uint8_t s = 0;
    while(b >= f) { s++; b -= f; }
    if(s==0)      { *d-- = U0; *d-- = U0; }
    else if(s==1) { *d-- = U1; *d-- = U1; }
    else          { *d-- = U0; *d-- = U1; }

    f /= 3;   // 27->9->3->1
  }
}

// encode 4 binary bits to ternary pattern
void b2t(uint8_t b, uint8_t *d) {
  uint8_t f = 0x08;

  while(f) {
    if(b & f)  { *d++ = U1; *d++ = U1; }
    else       { *d++ = U0; *d++ = U0; }
    f >>= 1;
  }
}

// encode single binary bit to ternary pattern
void b2t(bool b, uint8_t *d) {
  if(b) { *d++ = U1; *d++ = U1; }
  else  { *d++ = U0; *d++ = U0; }  
}

// each command is sent twice with a pause of 4.2ms
void mm_send(uint8_t a, bool f, uint8_t s) {
  uint8_t cmd[18];

  a2t(a, cmd+0);     // encode locomotion address
  b2t(f, cmd+8);     // encode function state
  b2t(s, cmd+10);    // encode speed
  
  // this sends the command as 18 6 bit words over UART
  mm_tx(cmd, sizeof(cmd));
  // the command should be repeated after 3 trinary unit times = 3 * 416µs = 1.248ms
  // the next command can then be sent after 4.2ms (~10 trinary unit times)  
  _delay_us(1642);
  mm_tx(cmd, sizeof(cmd));
}

void loop() {
  static uint8_t cnt = 0;
  static bool lights = false;

  if(cnt++ > 25) {
    cnt = 0;
    lights = !lights;
    digitalWrite(17, lights);
  }

  mm_send(L1,lights,2);   // drive L1 slowly with lights
  _delay_us(4428);
}
