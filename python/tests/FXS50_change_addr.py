import FXS50_FoxWire as sensor

if __name__ == "__main__":

    sensor.fx.init()

    # 1 - Escaneia sensores na rede
    # e exibe as configurações dos dispositivos encontrados
    scan = sensor.scan_info() # scaneia endereços

    addr_novo = 0x05

    # verifica se existe apenas um dispositivo na rede
    if( len(scan) == 1 ):

        addr = scan[0]
        
        # 2 - Altera o endereço do dispositivo conectado para addr_novo
        sensor.write_register( 0, addr, addr_novo )
        
        # 3 - Salva a alteração
        sensor.save(addr_novo)

        print( f'Dispositivo {addr} mudou o endereco para {addr_novo}' )



    # Encerra a conexão
    sensor.close()

