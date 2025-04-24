# import serial
# import time

# # Use Windows COM port
# ser = serial.Serial('COM13', 115200, timeout=1)

# print("Listening for 8-bit UART values every 1 second...")

# try:
#     while True:
#         byte = ser.read()
#         if byte:
#             value = byte[0]
#             print(f"Value: {value} (0x{value:02X})")
#         time.sleep(0.05)  # Small pause to prevent 100% CPU usage

# except KeyboardInterrupt:
#     print("\nExiting...")
#     ser.close()

import serial
import time

# Adjust this to match your serial port
SERIAL_PORT = 'COM10'  

BAUD_RATE = 9600

def read_16bit_random(ser):
    """Reads two bytes from serial and combines them into a 16-bit value."""
    high = ser.read()  # Read high byte
    if not high:
        return None
    low = ser.read()   # Read low byte
    if not low:
        return None

    # Combine high and low bytes
    return (high[0] << 8) | low[0]

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
            while True:
                rand_val = read_16bit_random(ser)
                if rand_val is not None:
                    print(f"Random Value: {rand_val} (0x{rand_val:04X})")
                else:
                    print("Timeout or incomplete data received.")
                # Delay is not necessary unless you want to throttle the output
                # time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    main()


# import serial

# ser = serial.Serial('COM13', 115200)

# while True:
#     data = ser.read(2)  # Expect 2 bytes per trigger
#     value = int.from_bytes(data, byteorder='big')
#     if value is not None:
#         print(f"Received: {data.hex()} -> {value}")
#     else:
#         print("---")


# import serial
# import time
# import random

# SERIAL_PORT = 'COM12'
# BAUD_RATE = 9600

# def send_16bit_random(ser):
#     """Generates and sends a 16-bit random number as two bytes (high, low)."""
#     rand_val = random.randint(0, 65535)
#     high_byte = (rand_val >> 8) & 0xFF
#     low_byte = rand_val & 0xFF

#     ser.write(bytes([high_byte, low_byte]))
#     print(f"Sent: {rand_val} (0x{rand_val:04X})")

# def main():
#     try:
#         with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
#             print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
#             time.sleep(2)  # Allow ESP32 to reset
#             while True:
#                 send_16bit_random(ser)
#                 time.sleep(1)  # Send every second
#     except serial.SerialException as e:
#         print(f"Serial error: {e}")
#     except KeyboardInterrupt:
#         print("\nStopped by user.")

# if __name__ == "__main__":
#     main()


# import serial

# # Adjust this to match your serial port
# SERIAL_PORT = 'COM10'
# BAUD_RATE = 9600

# def read_16bit_random(ser):
#     """Reads two bytes from serial and combines them into a 16-bit value."""
#     high = ser.read()
#     if not high:
#         return None
#     low = ser.read()
#     if not low:
#         return None

#     return (high[0] << 8) | low[0]

# def main():
#     last_val = None

#     try:
#         with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
#             print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")

#             while True:
#                 rand_val = read_16bit_random(ser)
#                 if rand_val is not None:
#                     if rand_val != last_val:
#                         print(f"Random Value: {rand_val} (0x{rand_val:04X})")
#                         last_val = rand_val
#                 # Optional: add small delay to reduce CPU usage
#                 # time.sleep(0.01)

#     except serial.SerialException as e:
#         print(f"Serial error: {e}")
#     except KeyboardInterrupt:
#         print("\nStopped by user.")

# if __name__ == "__main__":
#     main()
