/*
  Autor: Luis Felipe M. F. (luisf18)
  
  ---------------------------------------------------------
  Scanner FoxWire
  ---------------------------------------------------------

    Busca e printa os dispositivos encontrados.
  
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


#include "FoxWire.h"

#define FX_PIN A0

void setup() {
  
  Serial.begin(115200);

  FoxWire_init<FX_PIN>();

  Serial.println("scanning...");
  for(uint8_t addr=0;addr<=0x1F;addr++){
    uint8_t x = FoxWire_check<FX_PIN>(addr);
    if( x ){
      Serial.println("- Found: 0x" + String( addr, HEX ) );
    }
    delay(1);
  }
  Serial.println("end scanning");

}

void loop() {

}




