import serial
import time

# Use Windows COM port
ser = serial.Serial('COM13', 9600, timeout=1)

print("Listening for 8-bit UART values every 1 second...")

try:
    while True:
        byte = ser.read()
        if byte:
            value = byte[0]
            print(f"Value: {value} (0x{value:02X})")
        time.sleep(0.05)  # Small pause to prevent 100% CPU usage

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()