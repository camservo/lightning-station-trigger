import threading
import time

import serial

# Configuration for the serial connection
SERIAL_PORT = "/dev/tty.usbmodem33301"  # Change this to your serial port
BAUD_RATE = 115200

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def read_from_port(ser):
    while True:
        incoming_data = ser.readline().decode("utf-8").strip()
        if incoming_data:
            print(f"Received: {incoming_data}")


def write_to_port(ser):
    while True:
        message = input("Enter message to send: ")
        ser.write(message.encode("utf-8"))
        time.sleep(0.1)  # Small delay to ensure data is sent


def main():
    try:
        # Start the read thread
        thread = threading.Thread(target=read_from_port, args=(ser,))
        thread.start()

        # Start the write function
        write_to_port(ser)

    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial connection closed.")


if __name__ == "__main__":
    main()
