#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
char msg[6] = "hello";
RF24 radio(2, 15, 14, 12, 13);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void) {
  Serial.begin(115200);
  radio.begin();
  radio.setChannel(2);
  radio.setPayloadSize(7);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe);
}
void loop(void) {
  Serial.println("send ...");
  radio.write(msg, 6);
  delay(3000);
}