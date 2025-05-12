import FoxWire as fx
import time

fx.init( "COM10" )

print("scanning...")

for addr in range(0x00, 0x20):
    x = fx.CHECK(addr,False)
    if(x is not None):
        print("------------------------------------------")
        print(f"Found: 0x{addr:02X}")
        print(f"check answer: {x}")
        print(f"READ: { fx.command( addr, 13 ) }")

        reg0 = fx.read_register(addr, 0)
        print(f"REG[0] = 0x{reg0:02X}")

        id_l = fx.command( addr, fx.CMD_DEVICE_ID_L)
        id_h = fx.command( addr, fx.CMD_DEVICE_ID_H)
        device_id = (id_h << 8) | id_l
        print(f"DEVICE ID = 0x{device_id:04X}", end='')

        if device_id == 0x1:
            print(" [ Sensor FX-S50 ]")
        else:
            print(" [ unknown ]")

        lot_l = fx.command(addr, fx.CMD_LOT_L)
        lot_h = fx.command(addr, fx.CMD_LOT_H)
        lot = (lot_h << 8) | lot_l
        print(f"LOT = 0x{lot:04X}",end='')

        date_l = fx.command(addr, fx.CMD_LOT_DATE_L)
        date_h = fx.command(addr, fx.CMD_LOT_DATE_H)
        date = (date_h << 8) | date_l
        month = date % 12
        year = date // 12
        print(f" (Data: {month:02}/{year})")

        firmware_id  = fx.command(addr, fx.CMD_FIRMWARE_ID)
        firmware_ver = fx.command(addr, fx.CMD_FIRMWARE_VERSION)
        print(f"FIRMWARE = {firmware_id}.{firmware_ver}")

        foxwire_ver = fx.command(addr, fx.CMD_FOXWIRE_VERSION_ID)
        print(f"FoxWire Version = {foxwire_ver}")

        # Descarta primeiras leituras
        fx.command(addr, fx.CMD_MCU_VOLTAGE)
        fx.command(addr, fx.CMD_MCU_VOLTAGE)
        mcu_vcc_raw = fx.command(addr, fx.CMD_MCU_VOLTAGE)
        mcu_vcc = (mcu_vcc_raw + 128) * (125.0 / 8.0)
        print(f"MCU internal voltage = {mcu_vcc:.1f} mV")

        fx.command( addr, fx.CMD_MCU_RESET)
        print("restarting", end='')
        while (fx.CHECK(addr) is None ):
            print(".", end='')
            time.sleep(0.001)
        print()

        time.sleep(0.1)

        for i in range(1,32):
            fx.write_register(addr,i,1)

        print("------------------------------------------")
        print("REGISTERS WRITE 1")
        print("------------------------------------------")
        for i in range(4):
            for j in range(8):
                print( f"{fx.read_register(addr,j+8*i):02X}  ",end='' )
            print()
        print("------------------------------------------")

        fx.command_key(addr,fx.CMD_W_RESTORE_KEEP_ADDR)

        print("------------------------------------------")
        print("REGISTERS")
        print("------------------------------------------")
        for i in range(4):
            for j in range(8):
                print( f"{fx.read_register(addr,j+8*i):02X}  ",end='' )
            print()
        print("------------------------------------------")

    time.sleep(0.001)

print("end scanning")