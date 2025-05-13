import serial
import time

# Configurações da porta serial
ser = None

# lista de comandos READ do core FoxWire
CMD_DEVICE_ID_L        = 0
CMD_DEVICE_ID_H        = 1
CMD_LOT_L              = 2
CMD_LOT_H              = 3
CMD_LOT_DATE_L         = 4
CMD_LOT_DATE_H         = 5
CMD_FOXWIRE_VERSION_ID = 6
CMD_FIRMWARE_ID        = 7
CMD_FIRMWARE_VERSION   = 8
CMD_REQUEST_WRITE      = 9
CMD_MCU_RESET          = 10
CMD_MCU_VOLTAGE        = 11
CMD_MCU_TEMPERATURE    = 12


# lista de comandos WRITE do core FoxWire
CMD_W_SAVE = 1
CMD_W_RESTORE = 2
CMD_W_RESTORE_KEEP_ADDR = 3

DEVICES = [
    
    # Sensors
    {'name': 'FX-S50',  'id': 0x0001},
    {'name': 'FX-S53',  'id': 0x0004},
    
    {'name': 'FX-S200', 'id': 0x0010},
    {'name': 'FX-S203', 'id': 0x0011},
    
    {'name': 'FX-IR',   'id': 0x0020},
    
    # ESCs Brushed
    {'name': 'FX-ESC',     'id': 0x0100},
    {'name': 'FX-ESC-1X3', 'id': 0x0101},

    {'name': 'FX-ESC-2X3', 'id': 0x0110},
    {'name': 'FX-ESC-2X3', 'id': 0x0111},

    # ESCs Brushless
    {'name': 'FX-ESC-BLDC', 'id': 0x0130},
    
]

# =====================================================
# Init, Close and logif
# =====================================================

def init( port = 'COM16', Ser = None ):
    global ser
    if Ser:
        ser = Ser
    else:
        ser = serial.Serial(port, 115200, timeout=0.001)

def close():
    ser.close()

def logif(txt,log):
    if(log):
        print(txt)

# =====================================================
# UART Halfduplex Read and Write Function
# =====================================================

# envia bytes
def send( data, recive_size, log = False ):
    try:
        recive_size += len(data)
        ser.reset_input_buffer()
        ser.write(data)
        ser.flush()  # Garante que os dados foram enviados
        time.sleep(0.001)

        response = ser.read(recive_size)
        if log:
            print(f"\033[96mSEND[{len(data)}]:\033[0m {data} -> \033[92m{response}\033[0m")
        return response
    except serial.SerialException as e:
        print(f"\033[91mErro na comunicação serial:\033[0m {e}")
    return None

# =====================================================
# Package Functions
#  Check, Read and Write
# =====================================================

# pacote CHECK
def pack_check( addr, log = False ):
    byte1 = 0x80 + (addr&0x1F)
    response = send( [byte1], 1 )
    if response:
        if(log):
            for r in response: print(f"R - {r}")
        if( len(response) > 1 ):
            if( response[1]&0x1F == addr ):
                logif( "connected", log )
                return (3&(response[1]>>5))
            else:
                logif( "Incorrect", log )
        else:
            logif( "Not connected!", log )
    else:
        logif( "Nenhuma resposta recebida.", log )
    return None

# pacote READ
def pack_read( device_addr, addr, log = False ):
    byte1 = 0x80 | 0x1 << 5 | (device_addr&0x1F)
    response = send( [byte1,addr], 2 )
    if response:
        if(len( response ) == 3):
            logif(f"R - {response[2]}",log)
            return response[2]
    return None

# pacote WRITE
def pack_write( device_addr, addr, val, log = False ):
    byte1 = 0x80 | 0x2 << 5 | (device_addr&0x1F)
    response = send( [byte1,addr,val], 3 )
    if response:
        if(len( response ) == 4):
            logif(f"R - {response[3]}",log)
            return response[3]
    return None

# =====================================================
# Utils
#  Scan and checksum
# =====================================================

# busca dispositivos
def scan( log = False ):
    if(log):
        print( f"Scanning..." )
    found = []
    for i in range(0x1F+1):
        if( pack_check(i,False) is not None ):
            if(log):
                print( f"- {hex(i)}" )
            found += [i]
    return found

def checksum(v):
    return ( 3 & ( (v&1) + ((v>>1)&1) + ((v>>2)&1) + ((v>>3)&1) + ((v>>4)&1) ) )

def get_device_name( id ):
    for d in DEVICES:
        if( d['id'] == id ):
            return d['name']
    return 'unknown'

def hexdump( data ):
    for i in range(0, len(data), 8):
        group = data[i:i+8]
        start = i
        end = i + len(group) - 1
        # Endereço
        print(f"0x{start:02X} - 0x{end:02X}   ", end='')

        # HEX
        for v in group:
            print(f"{v:02X} ", end='')
        print("  ", end='')  # Espaço entre grupos

        # DEC
        for v in group:
            print(f"{v:3d} ", end='')
        print("  ", end='')  # Espaço entre grupos

        # CHAR
        for v in group:
            c = chr(v) if 32 <= v < 127 else '.'
            print(f" {c} ", end='')

        print()  # Próxima linha

# =====================================================
# Main Functions
# =====================================================

# ----------------------------------------------------
# Registers
#   Read and Write
# ----------------------------------------------------
# Registradores
def read_register( addr, reg, log = False ):
    temp  = 0x80 | (checksum(reg)<<5) | (reg&0x1F)
    return pack_read( addr, temp )

def write_register( addr, reg, value, log = False ):
    temp  = 0x80 | (checksum(reg)<<5) | (reg&0x1F)
    return pack_write( addr, temp, value )

# ----------------------------------------------------
# Commands
#   simple Read and Write or with key
# ----------------------------------------------------
def command( addr, cmd, value = None, log = False ):
    temp  = (checksum(cmd)<<5) | (cmd&0x1F)
    if(value):
        return pack_write( addr, temp, value )
    else:
        return pack_read( addr, temp )

def command_key( addr, cmd, log = False ):
    key = command( addr, CMD_REQUEST_WRITE )
    if( key ):
        data = 0xff&(~key)
        return command( addr, cmd, data )
    return 0

# =====================================================
# Basic Identify informations
# =====================================================

def get_device_id( device ):
    ID_H = command( device, CMD_DEVICE_ID_H )
    ID_L = command( device, CMD_DEVICE_ID_L )
    return ( 0xFFFF&( ID_H<<8 | ID_L ) )

def get_lot_id( device ):
    LOT_H = command( device, CMD_LOT_H )
    LOT_L = command( device, CMD_LOT_L )
    return ( 0xFFFF&( LOT_H<<8 | LOT_L ) )

def get_firmware_ver_str( device ):
    FIRMWARE_ID = command( device, CMD_FIRMWARE_ID )
    FIRMWARE_VER = command( device, CMD_FIRMWARE_VERSION )
    return f"{FIRMWARE_ID}.{FIRMWARE_VER}"

def get_foxwire_ver( device ):
    return command( device, CMD_FOXWIRE_VERSION_ID )

def get_lot_date_str( device ):
    DATE_H = command( device, CMD_LOT_DATE_H )
    DATE_L = command( device, CMD_LOT_DATE_L )
    DATE = ( 0xFFFF&( (DATE_H<<8) | DATE_L ) )
    DATE_M = DATE%12
    DATE_Y = DATE//12
    return f"{DATE_M:02}/{DATE_Y:04}"

def get_mcu_voltage( device ):
    V_8BITS = command( device, CMD_MCU_VOLTAGE )
    return (V_8BITS+128)*(125.0/8.0)

def get_mcu_temperature( device ):
    TEMP = command( device, CMD_MCU_TEMPERATURE )
    return TEMP

def dump(device):
    id = get_device_id(device)
    get_device_name( id )
    print(f"{'Addr:':<18} 0x{device:02X}")
    print(f"{'FoxWire Version:':<18} {get_foxwire_ver(device)}.0")
    print(f"{'Device Id:':<18} 0x{id:04X} ({get_device_name(id)})")
    print(f"{'Lot Id:':<18} 0x{get_lot_id(device):04X}")
    print(f"{'Lot date:':<18} {get_lot_date_str(device)}")
    print(f"{'Firmware Version:':<18} {get_firmware_ver_str(device)}")
    print(f"{'MCU voltage:':<18} {get_mcu_voltage(device)} mV")
    print(f"{'MCU temp:':<18} {get_mcu_temperature(device)} °C")

def mem_dump(device):
    return [ read_register( device, i ) for i in range(32)]

# =====================================================
# Tests
# =====================================================

if __name__ == "__main__":
    init()
    scan(True)
    ser.close()
