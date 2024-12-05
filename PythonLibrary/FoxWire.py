import serial
import time

# Configurações da porta serial
#arduino_port = 'COM10'  # Substitua pela porta correta (ex: COM3 no Windows ou /dev/ttyUSB0 no Linux)
arduino_port = 'COM16'  # Substitua pela porta correta (ex: COM3 no Windows ou /dev/ttyUSB0 no Linux)
baud_rate = 115200  # Taxa de comunicação. Certifique-se de que o Arduino está usando a mesma taxa
ser = None

def init( port = 'COM16' ):
    global ser
    global arduino_port
    arduino_port = port
    # Inicia a comunicação serial
    ser = serial.Serial(arduino_port, baud_rate, timeout=0.05)

def LOG( txt, log ):
    if(log):
        print( txt )

# envia bytes
def send( data, recive_size, log = False ):
    try:
        recive_size += len(data)
        ser.write(data)
        time.sleep(0.005*len(data))
        response = ser.read(recive_size)
        if(log):
            print(f"SEND[{recive_size}]: {data} -> {response}")
        return response
    except serial.SerialException as e:
        print(f"Erro na comunicação serial: {e}")
    return None

# pacote CHECK
def CHECK( addr, log = False ):
    byte1 = 0x80 + (addr&0x1F)
    response = send( [byte1], 1 )
    if response:
        if(log):
            for r in response: print(f"R - {r}")
        if( len(response) > 1 ):
            if( response[1]&0x1F == addr ):
                LOG( "connected", log )
                return 1
            else:
                LOG( "Incorrect", log )
        else:
            LOG( "Not connected!", log )
    else:
        LOG( "Nenhuma resposta recebida.", log )
    return None

# pacote READ
def READ( device_addr, addr, log = False ):
    byte1 = 0x80 | 0x1 << 5 | (device_addr&0x1F)
    response = send( [byte1,addr], 2 )
    if response:
        if(len( response ) == 3):
            LOG(f"R - {response[2]}",log)
            return response[2]
    return None

# pacote WRITE
def WRITE( device_addr, addr, val, log = False ):
    byte1 = 0x80 | 0x2 << 5 | (device_addr&0x1F)
    response = send( [byte1,addr,val], 3 )
    if response:
        if(len( response ) == 4):
            LOG(f"R - {response[3]}",log)
            return response[3]
    return None

# busca dispositivos
def scan( log = False ):
    if(log):
        print( f"Scanning..." )
    found = []
    for i in range(0x1F+1):
        if(CHECK(i,False)):
            if(log):
                print( f"[Found][{hex(i)}]" )
            found += [i]
    if(log):
        print( f"end scanning" )
    return found

if __name__ == "__main__":
    init()
    scan(True)
    ser.close()
