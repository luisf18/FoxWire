import FoxWire as fx

fx.init("COM10")
addr = fx.scan(True)

if( len(addr) ):
    addr = addr[0]
    
    #r = fx.command( addr, fx.CMD_MCU_RESET )
    
    #r = fx.command_key( addr, fx.CMD_W_RESTORE, True )
    #print( f"ok? {r}" )

    #r = fx.command_key( addr, fx.CMD_W_RESTORE_KEEP_ADDR, True )
    
    for i in range(0,10):
        r = fx.command( addr, i )
        if( r is not None ):
            print( f"INFO[{i}]: {r}" )

    for i in range(1,32):
        r = fx.read_register( addr, i )
        print( f"R[{i}]: {r} {chr(r)}", end="" )
        r = fx.write_register( addr, i, 0 )
        if( r is None ):
            print( " [X]" )
        else:
            print( f" W[{i}]: {r} {chr(r)}" )
    
    #r = fx.command_key( addr, fx.CMD_W_SAVE, True )
    #print( f"ok? {r}" )

fx.ser.close()