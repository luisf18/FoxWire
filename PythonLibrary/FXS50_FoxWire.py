import FoxWire as fx

# -------------------------------------
# Comandos
# -------------------------------------

REGISTERS = {
    "ADDR": 0x00,
    "CTRL": 0x01,
    "LED_HZ": 0x02,
    "LED_POWER": 0x03,
    "FILTER_LEN": 0x04,
    "FILTER_TRIG": 0x05,
    "READ": 0x06
}

# -------------------------------------
# Comandos
# -------------------------------------
CMD_DEVICE_ID             = 0x00
CMD_FIRMWARE_ID           = 0x01
CMD_FOXWIRE_VERSION_ID    = 0x02
CMD_READ                  = 0x03
CMD_RESET                 = 0x04
CMD_REQUEST_WRITE         = 0x05
CMD_W_RESTORE             = 0x01
CMD_W_RESTORE_KEPP_ADDR   = 0x02
CMD_W_SAVE                = 0x03

#-------------------------------------------------------------------
# Funções principais
#-------------------------------------------------------------------

def checksum(v):
    return 0x3 & ( (v&1) + ((v>>1)&1) + ((v>>2)&1) + ((v>>3)&1) + ((v>>4)&1) )

# Registradores
def read_register( addr, reg, log = False ):
    temp  = 0x80 | (checksum(reg)<<5) | (reg&0x1F)
    return fx.READ( addr, temp )

def write_register( addr, reg, value, log = False ):
    temp  = 0x80 | (checksum(reg)<<5) | (reg&0x1F)
    return fx.WRITE( addr, temp, value )

# Comandos
def command( addr, cmd, value = None, log = False ):
    temp  = (checksum(cmd)<<5) | (cmd&0x1F)
    if(value):
        return fx.WRITE( addr, temp, value )
    else:
        return fx.READ( addr, temp )

def command_key( addr, cmd, log = False ):
    key = command( addr, CMD_REQUEST_WRITE )
    if( key ):
        data = 0xff&(~key)
        fx.LOG(f"key [{data}]",log)
    return 0

#-------------------------------------------------------------------
# Principais comandos
#-------------------------------------------------------------------

def read_registers( addr ):
    print(f"REGISTERS: \t\tDEC\tHEX")
    for register, reg in REGISTERS.items():
        value = read_register(addr,reg)
        if(not value is None):
            print(f"[Reg 0x{reg}] {register[:15]:<12}:\t{value}\t{hex(value)}")

def restore(n):
    return command_key( n, CMD_W_RESTORE )

def reset(n):
    return command( n, CMD_RESET )

def save(n):
    return command_key( n, CMD_W_SAVE )

def id(n):
    print( f"[ADDR: {hex(n)}]" )
    for i,txt in [ [CMD_DEVICE_ID,"DEVICE_ID:   "], [CMD_FIRMWARE_ID,"FIRMWARE_ID: "], [CMD_FOXWIRE_VERSION_ID,"FOXWIRE_ID:  "] ]:
        print( f"  [{txt}{command( n, i )}]" )

def scan_info():
    scan_result = fx.scan()
    for j in scan_result:
        #write_command_key( j, CMD_W_RESTORE )
        #print(f"=> {save(j)}")
        #time.sleep(2)
        #write_register( j, 7, ord('A') )
        print("--------------------------------------")
        id( j )
        for i in range(32):
            x = read_register( j, i )
            print(f"DEVICE[{j}][{i}]:\t{x} \t{chr(x)}")
        print("--------------------------------------")
    return scan_result

def close():
    fx.ser.close()

# -------------------------------------

if __name__ == "__main__":

    fx.init()

    # 1 - Escaneia sensores na rede
    # e exibe as configurações dos dispositivos encontrados
    scan_info()
    
    # 2 - Restaura as configurações do sensor 0x00
    #restore( 0 )

    # 3 - Altera o endereço do dispositivo 0x01 para 0x02
    #write_register( 0, 1, 2 )
    #save( 2 ) # salva a alteração

    # Encerra a conexão
    close()

