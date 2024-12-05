import FXS50_FoxWire as sensor

if __name__ == "__main__":

    sensor.fx.init()

    # 1 - Escaneia sensores na rede
    # e exibe as configurações dos dispositivos encontrados
    scan = sensor.fx.scan() # scaneia endereços

    # Endereço do dispositivo
    # caso não saiba coloque None
    #  para enviar para o menor endereço encontrado
    addr = None #0x05 # None

    # Caso seja None coloca o menor endereço encontrado
    if( addr is None and scan ):
        addr = scan[0]

    # Dicionario com os valores desejados
    # coloque as configurações que deseja alterar e o valor
    config = {
        "LED_HZ": 150,
        "LED_POWER": 100,
        "FILTER_LEN": 5,
        "FILTER_TRIG": 3
    }


    # verifica se o endereço esta disponivel
    if( addr in scan ):

        # 2 - altera as configurações
        for lable, value in config.items():
            sensor.write_register(
                addr,
                sensor.REGISTERS[lable],
                value
            )
        
        # 3 - Salva a alteração
        sensor.save(addr)

        #sensor.reset(addr)
        #print(sensor.REGISTERS)

        # 4 - Printa as configurações
        print( f'Configurações salvas:' )
        sensor.read_registers(addr)

    # 5 - Encerra a conexão
    sensor.close()

