import serial
from datetime import datetime

ser = serial.Serial('COM22', 9600, timeout=1)
print("Listening for ADC values...")

buffer = ""
try:
    while True:
        byte = ser.read()
        if byte:
            char = byte.decode('utf-8', errors='ignore')
            if char == '\n':
                line = buffer.strip()
                buffer = ""
                try:
                    value = int(line)
                    now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    inverted_value = 4095 - value
                    print(f"[{now}] ADC: {inverted_value}")
                except ValueError:
                    print("Parse error:", line)
            else:
                buffer += char

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
