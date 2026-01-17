import FoxWire as fx

if __name__ == "__main__":
    fx.init( "COM5" )

    print("scan1: ", fx.scan())
    for i in range(5):
        print( f"[{i}]: ", fx.read_register(0,i) )
        #print( f"[{i}]: ", fx.READ(0,i,log=True))
    print(fx.write_register(0,0,8))
    print("scan2: ", fx.scan())
    print(fx.command_key(8,1,1)) # save
    print("scan3: ", fx.scan())
    print(fx.command(8,10)) # reset