import serial
import sys

# ========= CONFIGURAÇÕES =========
PORT = "COM10"      # ajuste para sua porta (ex: COM5, COM7, etc)
BAUDRATE = 115200  # ajuste conforme necessário
TIMEOUT = 1        # segundos
# =================================

def printable_char(b):
    """Retorna caractere legível ou '.' se não imprimível"""
    if 32 <= b <= 126:
        return chr(b)
    return '.'

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta {PORT}: {e}")
        sys.exit(1)

    print(f"Conectado em {PORT} @ {BAUDRATE} bps")
    print("Pressione Ctrl+C para sair\n")

    try:
        while True:
            data = ser.read(1)  # lê 1 byte
            if not data:
                continue

            b = data[0]
            char = printable_char(b)
            dec = b
            hexv = f"0x{b:02X}"

            print(f"{char} {dec} {hexv}")

    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
