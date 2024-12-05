#include <stdint.h>

//---------------------------------------------------------------------------------
// Temporização
#define F_CPU 16000000  // Frequência do clock do sistema (ajuste conforme o seu MCU)
#define BAUD_RATE 115200
#define BIT_DELAY (F_CPU / BAUD_RATE) // Tempo por bit em ciclos de clock
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Manipulação de I/O
//#define FXW__PIN_TO_PORT_ADDR( p ) ( p <= 7 ? 0x0B : p <= 13 ? 0x05 : 0x08 )
#define FXW__PIN_TO_PORT( pin ) ( pin <= 7 ? PORTD : pin <= 13 ? PORTB : PORTC )
#define FXW__PIN_TO_DDR( pin ) ( pin <= 7 ? DDRD : pin <= 13 ? DDRB : DDRC )
#define FXW__PIN_TO_PIN( pin ) ( pin <= 7 ? PIND : pin <= 13 ? PINB : PINC )
#define FXW__PIN_TO_BIT( pin ) ( pin <= 7 ? pin : pin <= 13 ? pin - 8 : pin - 14 )
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Tipos de pacote
#define FXW__CHECK   ( 0x80 | (0<<5) )
#define FXW__READ    ( 0x80 | (1<<5) )
#define FXW__WRITE   ( 0x80 | (2<<5) )
#define FXW__SPECIAL ( 0x80 | (3<<5) )
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
template< const uint8_t pin >
void FoxWire_init(){
  if( pin < 19 ){
    FXW__PIN_TO_PORT(pin) |= (1<<FXW__PIN_TO_BIT(pin));
    FXW__PIN_TO_DDR(pin)  &= ~(1<<FXW__PIN_TO_BIT(pin));
    delay(1);
  }
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
template< const uint8_t pin >
void FoxWire_write(uint8_t data) {

    if( pin > 19 ) return;

    asm volatile (
        
        // debug pulse
        //"sbi %[port], %[pinDebug] \n\t" // pulso no pino de debug
        //"cbi %[port], %[pinDebug] \n\t"

        "sbi %[mode], %[pin] \n\t"      // Clear bit (pinmode output)
        "cbi %[port], %[pin] \n\t"      // Clear bit (start bit)
        
        // Transmissão dos 8 bits de dados (LSB primeiro)
        "ldi r19, 8 \n\t"               //  [0]  Define contador de bits (8 bits)
        "send_bit: \n\t"    //  [0]  Desloca o próximo bit para Carry

        // delay de um ciclo
        "ldi r18, %[delay] \n\t"    // Carrega o valor do delay
        "bit_delay: dec r18 \n\t"   // Rótulo exclusivo para o delay dos bits
        "brne bit_delay \n\t"       // Aguarda o tempo do bit
        
        // envia 0 ou 1
        "lsr %[data] \n\t"          //  [0]  Desloca o próximo bit para Carry
        "brcs send_one \n\t"        //  [0]  Se Carry set, vai para enviar '1'
        "cbi %[port], %[pin] \n\t"  //  [0]  Envia '0' (Clear bit)
        "rjmp loop_end \n\t"        //  [0]  Salta para o delay
        "send_one: sbi %[port], %[pin] \n\t"  // Envia '1' (Set bit)

        "loop_end: \n\t"

        // pulse
        //"sbi %[port], %[pinDebug] \n\t" // pulso no pino de debug
        //"cbi %[port], %[pinDebug] \n\t"
        "nop \n\t"
        "nop \n\t"
        "nop \n\t"
        "nop \n\t"

        // loop
        "dec r19 \n\t"                  // Decrementa contador de bits
        "brne send_bit \n\t"            // Repete até que todos os bits sejam enviados

        "ldi r18, %[delay] \n\t"        // Carrega o valor do delay
        "last_bit_delay: dec r18 \n\t"      // Rótulo exclusivo para o delay do stop bit
        "brne last_bit_delay \n\t"          // Aguarda o tempo do stop bit
        
        // Configurar o stop bit (nível alto)
        "sbi %[port], %[pin] \n\t"      // Set bit (stop bit)
        "ldi r18, %[half_delay] \n\t"        // Carrega o valor do delay
        "stop_delay: dec r18 \n\t"      // Rótulo exclusivo para o delay do stop bit
        "brne stop_delay \n\t"          // Aguarda o tempo do stop bit
        
        // pulse
        //"sbi %[port], %[pinDebug] \n\t"
        //"cbi %[port], %[pinDebug] \n\t"
        
        // modo recebimento ----------------------------------------------------------------------
        "cbi %[mode], %[pin] \n\t"      // Clear bit (pinmode input)

        : 
        : 
          [port] "I" (_SFR_IO_ADDR(FXW__PIN_TO_PORT(pin))),
          [mode] "I" (_SFR_IO_ADDR(FXW__PIN_TO_DDR(pin))),
          [pin] "I" ( FXW__PIN_TO_BIT(pin) ),                   // Pino de saída
          //[pinDebug] "I" (5),                   // Pino de debug
          [data] "r" (data),                    // Dado a ser transmitido
          [delay] "M" ((BIT_DELAY/3)-2-2),        // Ajuste do delay (ciclos de clock)
          [half_delay] "M" ( (BIT_DELAY/6)-1 ) //(BIT_DELAY/6)-1)    // Ajuste do delay (ciclos de clock)
        : "r17", "r18", "r19","r20"             // Registradores usados
    );

}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// Fox READ
template< const uint8_t pin >
uint16_t FoxWire_read() {
  
  if( pin > 19 ) return 0;

  uint8_t ok = 0;
  uint8_t ret = 0;
  asm volatile (

    // modo recebimento ----------------------------------------------------------------------
    "cbi %[mode], %[pin] \n\t"  // Clear bit (pinmode input)
    "clr r17 \n\t"              //  [0]  Define contador de bits (8 bits)
    "ldi r19, 8 \n\t"           //  [0]  Define contador de bits (8 bits)
    "ldi r18, %[wait_delay] \n\t"         //  Define timeout
    
    // ----------------------------------------------------------------------
    // aguarda o pino ficar low
    "wait_fall:\n\t"  // nop
    "dec r18 \n\t"
    "breq end \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "sbic %[port_in], %[pin] \n\t"  // Verifica se o pino está em nível 0
    "rjmp wait_fall \n\t"
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // delay de meio periodo
    "ldi r18, %[half_delay] \n\t"    // Carrega o valor do delay
    "delay_init: dec r18 \n\t"      
    "brne delay_init \n\t"
    // ----------------------------------------------------------------------

    // "salvamento" ----------------------------------------------------------------------
    "capture_states: \n\t"

    // delay de um periodo
    "ldi r18, %[delay] \n\t"   // Carrega o valor do delay
    "_delay: dec r18 \n\t"      // Rótulo exclusivo para o delay do stop bit
    "brne _delay \n\t"          // Aguarda o tempo do stop bit

    "lsr r17 \n\t"                  // desloca o bit 
    "sbic %[port_in], %[pin] \n\t"  // Se o pino está em nível baixo, o próximo comando é pulado
    "ori r17, 0x80 \n\t"

    // pulse
    //"sbi %[port], %[pinDebug] \n\t"
    //"cbi %[port], %[pinDebug] \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"

    "dec r19 \n\t"                   // Decrementa o contador
    "brne capture_states \n\t"       // Se ainda não capturou 8 estados, repete

    // delay de um periodo
    "ldi r18, %[delay] \n\t"   // Carrega o valor do delay
    "_delay_end: dec r18 \n\t"      // Rótulo exclusivo para o delay do stop bit
    "brne _delay_end \n\t"          // Aguarda o tempo do stop bit
    
    "mov %0, r17 \n\t"
    "ldi %1, 1 \n\t"
    
    "end: \n\t"      // Rótulo exclusivo para o delay do stop bit

    // pulse
    //"sbi %[port], %[pinDebug] \n\t"
    //"cbi %[port], %[pinDebug] \n\t"

    : "=r" (ret), "=r" (ok)               // Saída: o byte de estados
    :
      [port_in] "I" (_SFR_IO_ADDR(FXW__PIN_TO_PIN(pin))),  // Porta de saída
      [port] "I" (_SFR_IO_ADDR(FXW__PIN_TO_PORT(pin))),
      [mode] "I" (_SFR_IO_ADDR(FXW__PIN_TO_DDR(pin))),
      [pin] "I" ( FXW__PIN_TO_BIT(pin) ),     // Pino de saída
      //[pinDebug] "I" (5),                   // Pino de debug
      [delay] "M" ((BIT_DELAY/3)-3),        // Ajuste do delay (ciclos de clock)
      [half_delay] "M" ((BIT_DELAY/6)-1),    // Ajuste do delay (ciclos de clock)
      [wait_delay] "M" (200) // ( 2*(BIT_DELAY/3) ), // delay inicial de aguardo
    : "r17", "r18", "r19","r20"             // Registradores usados
  );

  return ( (ok<<8) | ret );
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// FOX CHECK
template< const uint8_t pin >
uint8_t FoxWire_check(uint8_t addr) {
  cli();
  FoxWire_write<pin>( FXW__CHECK | addr );
  uint16_t x = FoxWire_read<pin>();
  sei();
  if( (x&0x100) && ((x&0x1F)==addr) ){
    return ((x>>5)&0x07) + 1;
    //Serial.println( "Addr [0x" + String(addr[i]) + "]: 0b" + String(x&0xFF,BIN) + "\t0x" + String(x&0xFF,HEX) );
  }
  return 0;
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// FOX READ
template< const uint8_t pin >
uint8_t FoxWire_pack_read(uint8_t addr, uint8_t data) {
  cli();
  FoxWire_write<pin>( FXW__READ | addr ); // inicia a comunicação
  delayMicroseconds(12);
  FoxWire_write<pin>( data );
  uint8_t ret = FoxWire_read<pin>();
  sei();
  return ret;
}
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// FOX WRITE
template< const uint8_t pin >
uint8_t FoxWire_pack_write(uint8_t addr, uint8_t data1, uint8_t data2 ) {
  cli();
  FoxWire_write<pin>( FXW__WRITE | addr ); // inicia a comunicação
  delayMicroseconds(12);
  FoxWire_write<pin>( data1 );
  delayMicroseconds(12);
  FoxWire_write<pin>( data2 );
  uint8_t ret = FoxWire_read<pin>();
  sei();
  return ret;
}
//---------------------------------------------------------------------------------



