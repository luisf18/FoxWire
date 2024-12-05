#include "FoxWire.h"


// -------------------------------------
// Comandos
// -------------------------------------
#define CMD_DEVICE_ID            0x00
#define CMD_FIRMWARE_ID          0x01
#define CMD_FOXWIRE_VERSION_ID   0x02
#define CMD_READ                 0x03
#define CMD_RESET                0x04
#define CMD_REQUEST_WRITE        0x05

#define CMD_W_RESTORE            0x01
#define CMD_W_RESTORE_KEPP_ADDR  0x02
#define CMD_W_SAVE               0x03


// funções
template< const uint8_t pin >
uint8_t FXsensor_register_read( uint8_t addr, uint8_t reg );
template< const uint8_t pin >
uint8_t FXsensor_command_read( uint8_t addr, uint8_t reg );
template< const uint8_t pin >
uint8_t FXsensor_register_write( uint8_t addr, uint8_t reg, uint8_t data );
template< const uint8_t pin >
uint8_t FXsensor_command_write( uint8_t addr, uint8_t reg, uint8_t data );
template< const uint8_t pin >
uint16_t FXsensor_write_command_key( uint8_t addr, uint8_t cmd );

// -------------------------------------------------------------------------------------

template< const uint8_t pin >
uint8_t FXsensor_info( uint8_t addr ){
  Serial.println("\n------------------------" );
  Serial.println("ADDR: 0x" + String( addr, HEX ) );
  Serial.println("------------------------" );
  if( ! FoxWire_check<pin>(addr) ){
    Serial.println("READ FAIL!" );
    Serial.println("------------------------" );
    return 0;
  }
  Serial.println("REG   DEC  HEX  ASCII " );
  Serial.println("------------------------" );
  for( int i=0;i<16;i++ ){
    uint8_t reg = FXsensor_register_read<pin>( addr, i );
    char txt[200];
    sprintf( txt, "0x%02X: %3d  0x%02X  %c ", i, reg, reg, (char)reg );
    Serial.println( txt );
  }
  Serial.println("------------------------\n" );
  return true;
}

#define FXsensor_checksum(v) ( 0x3&( (v&1) + ((v>>1)&1) + ((v>>2)&1) + ((v>>3)&1) + ((v>>4)&1) ) )

// Read a register
template< const uint8_t pin >
uint8_t FXsensor_register_read( uint8_t addr, uint8_t reg ){
  return FoxWire_pack_read<pin>( addr, 0x80 | (FXsensor_checksum(reg)<<5) | (reg&0x1F) );
}

// Commando do tipo read
template< const uint8_t pin >
uint8_t FXsensor_command_read( uint8_t addr, uint8_t reg ){
  return FoxWire_pack_read<pin>( addr, (FXsensor_checksum(reg)<<5) | (reg&0x1F) );
}

// Write a register
template< const uint8_t pin >
uint8_t FXsensor_register_write( uint8_t addr, uint8_t reg, uint8_t data ){
  return FoxWire_pack_write<pin>( addr, 0x80 | (FXsensor_checksum(reg)<<5) | (reg&0x1F), data );
}

// Write a register
template< const uint8_t pin >
uint8_t FXsensor_command_write( uint8_t addr, uint8_t cmd, uint8_t data ){
  return FoxWire_pack_write<pin>( addr, (FXsensor_checksum(cmd)<<5) | (cmd&0x1F), data );
}

// Write a register
template< const uint8_t pin >
uint16_t FXsensor_write_command_key( uint8_t addr, uint8_t cmd ){
  uint8_t key = FXsensor_command_read<pin>( addr, CMD_REQUEST_WRITE );
  if( key == 0 ){ // falha
    return 0;
  }
  key = 0xff&(~key);
  delayMicroseconds(500);
  return 0x100 | FXsensor_command_write<pin>( addr, cmd, key );
}

