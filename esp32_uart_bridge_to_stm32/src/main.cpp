#include <Arduino.h>


#define STM_START 14
#define BOOT0     4
#define TX_STM    17
#define RX_STM    16
#define TX_USB    1
#define RX_USB    3

void setup() {
    // Serial.begin(9600);
    // while(!Serial);
    // Serial2.begin(9600, SERIAL_8N1 , 16, 17, false);
    // while(!Serial2);

    pinMode(BOOT0, OUTPUT); // gd32- BOOT0
    digitalWrite(BOOT0, 1);
    pinMode(STM_START, OUTPUT); // STM_START
    delay(250);
    digitalWrite(STM_START, 1);

    pinMode(1, OUTPUT); // TXD0
    pinMode(3, INPUT); // RXD0
    pinMode(TX_STM, OUTPUT); // TX1
    pinMode(RX_STM, INPUT); // RX1
}


void loop() {
    digitalWrite(TX_STM, digitalRead(RX_USB));
    digitalWrite(TX_USB, digitalRead(RX_STM));
}
