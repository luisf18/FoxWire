/*
  Autor: Luis Felipe M. F. (luisf18)
  
  ---------------------------------------------------------
  Leitura de sensores FX-S50 usando FoxWire
  ---------------------------------------------------------

    Associa um pino com cada sensor detectado na rede.
  Caso o sensor esteja detectando o led acende.
  
  Placas compativeis:
    Atmega328
      - Arduino UNO
      - Arduino Nano
      - Arduino Micro
  
  Circuito:
    Vcc  ----------------------- Vcc
    Pino -----[Resistor 1k]----  Fx ... dispositivos ...
    GND  ----------------------- GND

*/

#include "FXS50.h"

#define FX_PIN A0       // pino FX
#define SENSORS_COUNT 3 // numero de sensores

uint8_t sensors_detected = 0;
uint8_t sensors_addr[SENSORS_COUNT];

void setup() {
  
  Serial.begin(115200);

  //pinMode LEDs
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(2,OUTPUT);

  FoxWire_init<FX_PIN>();

  for(uint8_t i=0;(i<=0x1F) && (sensors_detected<SENSORS_COUNT);i++){
    uint8_t x = FoxWire_check<FX_PIN>(i);
    if( x ){
      Serial.println("- Found: 0x" + String( i, HEX ) + " pino do LED = " + String(3+sensors_detected) );
      sensors_addr[sensors_detected] = i;
      sensors_detected++;
    }
  }

}

void loop() {
  for(uint8_t i=0; i<sensors_detected ;i++){
    uint8_t x = FoxWire_check<A0>(sensors_addr[i]);
    digitalWrite( 3+i, x == 2 );
  }
  delay(1);
}




