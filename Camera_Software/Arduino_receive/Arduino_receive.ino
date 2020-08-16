#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
char msg[6];
RF24 radio(7,8);
const uint64_t pipe = 0xE8E8F0F0E1LL;
void setup(void) {
 Serial.begin(115200);
 radio.begin();
 radio.setChannel(120);
 radio.setPayloadSize(7);
 radio.setDataRate(RF24_250KBPS);
 radio.setPALevel(RF24_PA_MAX);
 radio.openReadingPipe(1,pipe);
 radio.startListening();
}
int count = 0;
uint32_t currentMillis = 0;
void loop(void){
 if (radio.available()){  
     radio.read(msg, 6);      
     Serial.print(msg);
     Serial.print("   ");
    count++;
    if((millis() - currentMillis) >=10000){
        Serial.println();
        Serial.println(count);
        currentMillis = millis();
        count = 0;
    }
 }
}
