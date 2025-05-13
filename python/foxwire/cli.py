import foxwire as fx
import argparse
import serial.tools.list_ports
import sys

def main():

    # =========================================================
    # CMD colors
    # =========================================================
    BLUE   = "\033[34m"
    RED    = "\033[91m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RESET  = "\033[0m"

    def log_ok(txt=''):
        print(GREEN+"[ok]"+txt+RESET)
    def log_fail(txt=''):
        print(RED+"[fail]"+txt+RESET)
    def log_erro(txt=''):
        print(RED+"[erro]"+txt+RESET)
    def log_status(x):
        if(x):
            log_ok()
        else:
            log_erro()
    
    
    # =========================================================
    # Argument Parser
    # =========================================================
    
    #parser = argparse.ArgumentParser(description="FoxWire CLI")
    parser = argparse.ArgumentParser(
        description="FoxWire CLI",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Argumentos de ação
    parser.add_argument("-D", "--devices_scan", action='store_true', help="scan devices")
    parser.add_argument('-t', '--test', action='store_true')
    
    # Connection Requirements
    cnn_req = parser.add_argument_group('Connection Requirements')
    cnn_req.add_argument('-d', '--device', metavar=('ADDR') )
    cnn_req.add_argument('-c', '--com', help='Serial Port (ex: COM3 ou /dev/ttyUSB0) ou \'list\' para listar')

    # Registers
    cmd_reg = parser.add_argument_group('Registers')
    cmd_reg.add_argument('-R', '--read_register',  metavar=('[ADDR]') )
    cmd_reg.add_argument('-W', '--write_register', nargs=2, metavar=('[ADDR]','[VALUE]') )
    cmd_reg.add_argument('-H', '--memmory_hexdump', action='store_true')

    # Commands
    cmd_cmd = parser.add_argument_group('Commands')
    cmd_cmd.add_argument('-C', '--command', metavar=('[CMD]'))
    cmd_cmd.add_argument('-C2', '--command2', nargs=2, metavar=('[CMD][ARG]'))
    cmd_cmd.add_argument('-K', '--command_key', metavar=('[CMD]'))
    cmd_cmd.add_argument('-r', '--reset', action='store_true', help='device reset')
    cmd_cmd.add_argument('-s', '--save', action='store_true', help='[key] save actual configuration in flash')
    cmd_cmd.add_argument('-a', '--restore', action='store_true', help='[key] Restore to default configurations')
    cmd_cmd.add_argument('-A', '--restore_keep_addr', action='store_true', help='[key] Restore to default configurations except addr')
    
    # Packages
    cmd_packages = parser.add_argument_group('Basic Packages')
    cmd_packages.add_argument('-check', '--pack_check', action='store_true')
    cmd_packages.add_argument('-p_read', '--pack_read')
    cmd_packages.add_argument('-p_write', '--pack_write', nargs=2, metavar=('[ADDR]','[VALUE]') )

    args = parser.parse_args(sys.argv[1:])

    # =========================================================
    # Execute - [1] Connect to Serial COM port
    # =========================================================
    if args.com is None:
        #print( "Serial Port não informada " )
        ports = list(serial.tools.list_ports.comports())
        if ports is None:
            log_erro(" any Serial Port available!")
            sys.exit(0)
        else:
            args.com = ports[0].device
            #for p in ports:
            #    print(f"- {p.device}")
    else:
        if args.com == 'list':
            ports = list(serial.tools.list_ports.comports())
            if ports:
                print("Available Serial Ports:")
                for p in ports:
                    print(f"- {p.device}")
            else:
                print("Nenhuma porta serial encontrada.")
            sys.exit(0)
    
    print(f"{BLUE}Serial Port: {args.com}{RESET} ",end='')

    try:
        fx.init(port=args.com)
        #ser = serial.Serial(args.com, baudrate=115200, timeout=1)
        log_ok()
    except serial.SerialException as e:
        if "FileNotFoundError" in str(e) or "could not open port" in str(e):
            print(f"{RED}Error:{RESET} Port '{args.com}' not found.")
            print("Make sure the device is connected and the port name is correct.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  - {p.device}")
        else:
            print(f"{RED}Error:{RESET} Could not open port '{args.com}': {e}")
        sys.exit(1)
    except Exception as e:
        print(f"{RED}Unexpected error:{RESET} {e}")
        sys.exit(1)

    # =========================================================
    # Execute - [2] Connect to Device or Scan Devices
    # =========================================================

    # scan
    if( args.devices_scan ):
        fx.scan(True)
        fx.close()
        sys.exit(0)
    
    # DEVICE
    if args.device is None:
        #print(f"{YELLOW}Scanning devices:{RESET} ", end='')
        #print(f"Scanning devices: ", end='')
        devices = fx.scan()
        if( len(devices) ):
            #print(f"{YELLOW}{devices}{RESET}")
            #print(devices)
            args.device = devices[0]
        else:
            log_erro( " no devices available" )
            fx.close()
            sys.exit(0)
    else:
        args.device = int( args.device )
    
    print(f"{BLUE}Device:      0x{args.device:02X}{RESET}  ", end='')

    if( fx.pack_check( args.device ) is None ):
        log_erro( " not found" )
        fx.close()
        sys.exit(0)
    else:
        log_ok()
    
    # =================================================
    # Execute - generic
    # =================================================
    if( args.test ):
        id = fx.get_device_id(args.device)
        name = fx.get_device_name( id )
        if( name == 'FX-S50' ):
            # read command
            print( f"Read: {fx.command( args.device, 13 )}" )
        fx.close()
        sys.exit(0)
    
    # =========================================================
    # Execute - REGISTER
    # =========================================================

    if( args.memmory_hexdump ):
        fx.hexdump( fx.mem_dump(args.device) )
        fx.close()
        sys.exit(0)

    if( args.read_register ):
        addr = int(args.read_register)
        if( addr >= 32 ):
            log_erro( " address must be less than 32!" )
        else:
            val = fx.read_register( args.device, addr )
            print( f"Reg[0x{addr:02X}] = {val} (0x{val:02X}) [{chr(val)}]" )
        fx.close()
        sys.exit(0)

    if( args.write_register ):
        addr, val = args.write_register
        addr = int(addr)
        val = int(val)
        val = fx.write_register( args.device, addr, val )
        print( f"Reg[0x{addr:02X}] = {val} (0x{val:02X}) [{chr(val)}]" )
        fx.close()
        sys.exit(0)

    # =================================================
    # Execute - COMMANDS
    # reset, save, restore, restore_keep_addr
    # =================================================

    # no key
    if( args.reset ):
        print(f"resseting: 0x{args.device:02X}",end=' ')
        log_status( 255 == fx.command(args.device,fx.CMD_MCU_RESET) )
        fx.close()
        sys.exit(0)
    
    # with key
    if( args.save ):
        print(f"save: 0x{args.device:02X}",end=' ')
        log_status( 255 == fx.command_key(args.device,fx.CMD_W_SAVE) )
        fx.close()
        sys.exit(0)
    if( args.restore ):
        print(f"restore: 0x{args.device:02X}",end=' ')
        log_status( 255 == fx.command_key(args.device,fx.CMD_W_RESTORE) )
        fx.close()
        sys.exit(0)
    if( args.restore_keep_addr ):
        print(f"restore_keep_addr: 0x{args.device:02X}",end=' ')
        log_status( 255 == fx.command_key(args.device,fx.CMD_W_RESTORE_KEEP_ADDR) )
        fx.close()
        sys.exit(0)
    
    if( args.command ):
        cmd = int(args.command)
        if( cmd >= 32 ):
            log_erro( " address must be less than 32!" )
        else:
            ans = fx.read_register( args.device, cmd )
            print( f"cmd {cmd} -> ans {ans}" )
        fx.close()
        sys.exit(0)
    if( args.command2 ):
        cmd, arg = args.command2
        cmd = int(cmd)
        arg = int(arg)
        if( cmd >= 32 ):
            log_erro( " address must be less than 32!" )
        else:
            ans = fx.command( args.device, cmd, arg )
            print( f"cmd {cmd} {arg} -> ans {ans}" )
        fx.close()
        sys.exit(0)
    if( args.command_key ):
        cmd = int(args.command_key)
        if( cmd >= 32 ):
            log_erro( " address must be less than 32!" )
        else:
            ans = fx.command_key( args.device, cmd )
            print( f"cmd_key {cmd} -> ans {ans}" )
        fx.close()
        sys.exit(0)
    
    # =================================================
    # Execute - packages
    # =================================================
    if( args.pack_check ):
        ans = fx.pack_check( args.device )
        if ans is None:
            log_fail( " no response" )
        else:
            # read command
            print( f"ans: {ans}" )
        fx.close()
        sys.exit(0)
    
    if( args.pack_read ):
        addr = int(args.pack_read)
        ans = fx.pack_read( args.device, addr )
        if ans is None:
            log_fail( " no response" )
        else:
            # read command
            print( f"arg: {addr} -> ans: {ans}" )
        fx.close()
        sys.exit(0)

    if( args.pack_write ):
        addr, val = args.pack_write
        addr = int(addr)
        val = int(val)
        ans = fx.pack_write( args.device, addr, val, 1 )
        if ans is None:
            log_fail( " no response" )
        else:
            # read command
            print( f"args: {addr}, {val} -> ans: {ans}" )
        fx.close()
        sys.exit(0)

    # =================================================
    # Execute - No args, so dump
    # =================================================
    fx.dump(args.device)

if __name__ == "__main__":
    main()