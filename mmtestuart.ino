/*
 * mmtestuart.ino
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
// This sketch uses the UART to generate the trinary pattern. The
// 26µs peek is exactly 1/416 of the 16Mhz base clock. The resulting bit
// pattern equals 38461.54 UART bit/s at 6N1 (one start bit, six data
// bits and one stop bit. The start bit is always low, the data bits can
// be specified and the stop is high. The UART signal must thus be 
// inverted. This doesn't matter since the differential transmission
// on the track consists of the data stream and it's inverted state, anyway.
// Thus sending a start bit and six 1 bits followed by the stop bit is
// the (inverse) pattern of the two patterns that form the trinary 0.
// A start bit and six 0 bits followed by the stop bit is
// the (inverse) pattern of the two patterns that form the trinary 1.

// Each trinary state thus is being sent as two 6N1 data words:
// Trinary 0 is being sent as words 63, 63
// Trinary 1 is being sent as words  0,  0
// Trinary 2 is being sent as words  0, 63

// uart 6n1 @ 38462 encoding of low-to-high or high-to-low event
#define U0  63   // start = 0, data = 111111, stop = 1
#define U1   0   // start = 0, data = 000000, stop = 1

void setup() {
  pinMode(30, OUTPUT);
  
  Serial1.begin(38462, SERIAL_6N1);
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
  Serial1.write(cmd, sizeof(cmd));
  // the command should be repeated after 3 trinary unit times = 3 * 416µs = 1.248ms
  // the next command can then be sent after 4.2ms (~10 trinary unit times)  

  // the delay is rather tricky since the Serial1.write starts the transfer in the
  // background which in turn causes the transmission to still be in progress after
  // Serial1.write returns
  _delay_us(4690);
  Serial1.write(cmd, sizeof(cmd));
}

void loop() {
  static uint8_t cnt = 0;
  static bool lights = false;

  if(cnt++ > 25) {
    cnt = 0;
    lights = !lights;
    digitalWrite(30, lights);
  }
  
  mm_send(L1,lights,2);   // drive L1 slowly with lights on/off
  _delay_us(7500);        // longer pause between (different) commands
}
