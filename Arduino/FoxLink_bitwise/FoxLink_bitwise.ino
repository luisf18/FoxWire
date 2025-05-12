// FOXLINK: RX -> 2 TX -> 3 [ 3 --|<|--- 2 ]
// Serial: RX -> PD0 TX->PD1
#define MASK_OUT 0b00000010
#define MASK_IN  0b00000101

void setup() {
  DDRD  = 0b00000010;
  //PORTD = MASK_IN;
}

//#define FXW_LOW() { PORTD &=   }

void loop() {
  // PORTB = 0xFF;
  // uint8_t x = PIND;
  // //Serial.print( x, BIN );
  // //Serial.print( " " );
  // //Serial.print( (x&MASK_IN) , BIN );
  // //Serial.print( " " );
  // //Serial.println( MASK_IN | ((x&1)<<3) | ((x&0b00000100)>>1), BIN );
  //PORTD = 0b00000101 | ((x&1)<<3);// | ((x&4)>>1);
  PORTD = 2&(PIND<<1);// | ((x&4)>>1);
  // PORTB = 0x00;
  // //delay(400);
}
