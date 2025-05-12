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

/* Funções principais:
   - FoxWire_check( device_addr )                           Checa se existes um dispositivo em um endereço
   - FoxWire_command( device_addr, comando )                Envia um comando
   - FoxWire_command_key( device_addr, comando )            Envia um comando com chave de acionamento
   - FoxWire_register_read( device_addr, reg_addr )         Le um registrador
   - FoxWire_register_write( device_addr, reg_addr, value ) Escreve em um registrador
  
  Pacotes:
   - FoxWire_check( device_addr )                 Pacote Check
   - FoxWire_READ( device_addr, arg1 )            Pacote Check
   - FoxWire_WRITE( device_addr, arg1, arg2 )     Pacote Check
   - FoxWire_SPECIAL                              Ainda não implementado

*/

#define FX_PIN A0

void setup() {
  
  Serial.begin(115200);

  FoxWire_init<FX_PIN>();

  Serial.println("scanning...");
  for(uint8_t addr=0;addr<=0x1F;addr++){
    uint8_t x = FoxWire_check<FX_PIN>(addr);
    if( x ){

      // -----------------------------------------------------------------------
      // Dump - Leituras de dados do dispositivo
      // -----------------------------------------------------------------------
      Serial.println( "------------------------------------------" );
      Serial.println("Found: 0x" + String( addr, HEX ) );
      
      // Le o conteudo do registrador 0 (geralmente armazena o endereço do dispositivo)
      uint8_t REG0 = FoxWire_register_read<FX_PIN>( addr, 0 );
      Serial.println("REG[0] = 0x" + String( REG0, HEX ) );
      
      // [>] Para mudar o endereço escreva no registrador 0 o novo endereço
      // [!] Cuidado para não colocar um endereço ja em uso por outro dispositivo
      //REG0 = FoxWire_register_write<FX_PIN>( addr, 0, 10 );
      //Serial.println("REG[0] = " + String( REG0 ) );
      //REG0 = FoxWire_register_read<FX_PIN>( addr, 0 );
      //Serial.println("REG[0] = " + String( REG0 ) );
      
      // Le o ID (identificador) do dispositivo
      uint8_t ID_L = FoxWire_command<FX_PIN>( addr, FXW_CMD_DEVICE_ID_L );
      uint8_t ID_H = FoxWire_command<FX_PIN>( addr, FXW_CMD_DEVICE_ID_H );
      uint16_t ID = ( ID_H << 8 ) | ID_L;
      Serial.print("DEVICE ID = 0x" + String( ID, HEX ) );
      
      switch( ID ){
        case 0x1: Serial.println(" [ Sensor FX-S50 ]" ); break;
        default: Serial.println(" [ unknown ]" ); break;
      }

      // Le o codigo do Lote
      uint8_t LOT_L = FoxWire_command<FX_PIN>( addr, FXW_CMD_LOT_L );
      uint8_t LOT_H = FoxWire_command<FX_PIN>( addr, FXW_CMD_LOT_H );
      uint16_t LOT = ( LOT_H << 8 ) | LOT_L;
      Serial.print("LOT = 0x" + String( LOT, HEX ) );
      
      // Le a data do Lote
      uint8_t DATE_L = FoxWire_command<FX_PIN>( addr, FXW_CMD_LOT_DATE_L );
      uint8_t DATE_H = FoxWire_command<FX_PIN>( addr, FXW_CMD_LOT_DATE_H );
      uint16_t DATE = ( DATE_H << 8 ) | DATE_L;
      uint16_t DATE_M = (DATE%12); // mês
      uint16_t DATE_Y = (DATE/12); // ano
      Serial.println(" ( Data: " + String( DATE_M ) + "/" + String( DATE_Y ) + " )" );
      // Aviso: Lote 1 do FXS50 está com a data errada :/ [ correto: 02/2025 ]

      // Le a vesão do Firmware
      uint8_t FIRMWARE_ID = FoxWire_command<FX_PIN>( addr, FXW_CMD_FIRMWARE_ID );
      uint8_t FIRMWARE_VER = FoxWire_command<FX_PIN>( addr, FXW_CMD_FIRMWARE_VERSION );
      Serial.println("FIRMWARE = " + String(FIRMWARE_ID) + "." + String(FIRMWARE_VER) );

      // Le a vesão do Firmware
      uint8_t FOXWIRE_VER = FoxWire_command<FX_PIN>( addr, FXW_CMD_FOXWIRE_VERSION_ID );
      Serial.println("FoxWire Version = " + String(FOXWIRE_VER) );

      // Le a vesão do Firmware
      // As primeiras leituras são menos precisas e geralmente erradas
      // São descartadas as duas primeiras
      // não precisa fazer sempre apenas quando fizer as primeiras leituras após o reset
      FoxWire_command<FX_PIN>( addr, FXW_CMD_MCU_VOLTAGE );
      FoxWire_command<FX_PIN>( addr, FXW_CMD_MCU_VOLTAGE );
      uint8_t MCU_VCC_RAW = FoxWire_command<FX_PIN>( addr, FXW_CMD_MCU_VOLTAGE );
      // conversão para milivolts = (R8+128)*(125/8)
      float MCU_VCC = (MCU_VCC_RAW+128)*(125.0/8.0);
      Serial.println("MCU internal voltage = " + String(MCU_VCC) + "mV" );

      // -----------------------------------------------------------------------
      // Outros comandos
      // -----------------------------------------------------------------------
      // [1] Comando para salvar as alterações na flash do dispositivo
      //FoxWire_command_key<FX_PIN>( addr, FXW_CMD_W_SAVE );

      // [2] Comando para reiniciar o dispositivo
      FoxWire_command<FX_PIN>( addr, FXW_CMD_MCU_RESET );
      Serial.print("restarting");
      while( FoxWire_check<FX_PIN>( addr ) == 0 ){
        Serial.print(".");
        delay(1);
      }
      Serial.println();
      delay(100);

      // [3] Comando para restauras as configurações de fabrica
      // Mantendo o endereço FoxWire e o nome
      //FoxWire_command_key<FX_PIN>( addr, FXW_CMD_W_RESTORE_KEPP_ADDR );
      
      Serial.println( "------------------------------------------" );
    }
    delay(1);
  }
  Serial.println("end scanning");
}

void loop() {

}




