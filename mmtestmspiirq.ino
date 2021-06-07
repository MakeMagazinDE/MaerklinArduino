/*
 * mmtest.ino
 */

#include <Arduino.h>

#define L0   78   // default for steam engines
#define L1   72   // default for diesel engines
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

// spi encoding of short and long pulses
#define U0  0b10000000
#define U1  0b11111110
#define UP  0b00000000    // state during pause

#define DEVICES  4
uint8_t addr[DEVICES] = { L0, L1, L2, L3 };

// each train message has a length of 9 trinary values. Each message is stored
// SPI encoded in a 18 byte buffer
uint8_t tx_buf[DEVICES][18];
uint8_t tx_state = 0, tx_cnt = 0, tx_dev = 0;

// send command two times with gap
ISR(USART1_UDRE_vect) {  
  switch(tx_state) {
    case 0:
    case 2:
      // first and second transmission of each message
      UDR1 = tx_buf[tx_dev][tx_cnt++];
      if(tx_cnt == 18) {
        tx_cnt = 0;
        tx_state++;
      }
      break;   
      
    case 1:
      // short pause between first and second transmission
      UDR1 = UP;
      if(++tx_cnt == 6) {
        tx_cnt = 0;
        tx_state++;
      }
      break;
      
    case 3:
      // long pause between different messages
      UDR1 = UP;
      if(++tx_cnt == 20) {
        tx_cnt = 0;
        tx_state++;
      }
      break;
  }

  // switch to next device
  if(tx_state == 4) {
    tx_state = 0;

    if(++tx_dev == DEVICES)
      tx_dev = 0;
  }
}

void setup() {
  Serial.begin(9600);
//  while(!Serial);

  Serial.println("MM MPSI IRQ");
  
  pinMode(17, OUTPUT);   // for debugging

  // initially all trains are stopped with lights on
  for(uint8_t i=0;i<DEVICES;i++)
    mm_set(i, true, 0);

  // init spi
  UCSR1C = (1<<UMSEL11)|(1<<UMSEL10); // set spi mode 0
  UCSR1B = (1<<TXEN1) | (1<<UDRIE1);  // enable transmitter and transmit buffer empty irq
  UBRR1 = 26*(F_CPU/2000000)-1;       // 26us bit length
  UDR1 = 0xff;
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

void mm_set(uint8_t index, bool f, uint8_t s) {
  // TODO: make sure we don't alter the buffer while it's
  // being transmitted
  a2t(addr[index], tx_buf[index]+0);
  b2t(f, tx_buf[index]+8);
  b2t(s, tx_buf[index]+10);
}

void loop() {
  Serial.println("ping ...");
  
  digitalWrite(17, HIGH);
  mm_set(1, true, 0);        // set engine 1 lights on and speed 0
  delay(500);
  digitalWrite(17, LOW);
  mm_set(1, false, 0);       // set engine 1 lights off and speed 0
  delay(500);
}
