import signal
import sys
import threading
import time

import serial

# Configuration for the serial connections
SENSOR_PORT = "/dev/tty.usbmodem33401"  # Sensor's serial port
CONTROLLER_PORT = "/dev/tty.usbmodem33301"  # Controller's serial port
BAUD_RATE = 115200  # Baud rate for both connections


def setup_serial_connection(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port}")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect on {port}: {e}")
        return None


def read_from_sensor(sensor_ser, controller_ser):
    while True:
        try:
            incoming_data = sensor_ser.readline().decode("utf-8").strip()
            if incoming_data:
                incoming_data = incoming_data + "\n"
                print(f"Received from sensor: {incoming_data}")
                controller_ser.write(incoming_data.encode("utf-8"))

        except Exception as e:
            print(f"Error during sensor read/write: {e}")
            break


def read_from_controller(controller_ser):
    while True:
        try:
            incoming_data = controller_ser.readline().decode("utf-8").strip()
            if incoming_data:
                print(f"Received from controller: {incoming_data}")
        except Exception as e:
            print(f"Error during controller read: {e}")
            break


def signal_handler(sig, frame):
    print("You pressed Ctrl+C!")
    if sensor_ser and sensor_ser.is_open:
        sensor_ser.close()
    if controller_ser and controller_ser.is_open:
        controller_ser.close()
    sys.exit(0)


def main():
    global sensor_ser, controller_ser
    sensor_ser = setup_serial_connection(SENSOR_PORT, BAUD_RATE)
    controller_ser = setup_serial_connection(CONTROLLER_PORT, BAUD_RATE)

    if sensor_ser is None or controller_ser is None:
        return

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Start threads for reading from both sensor and controller
        sensor_thread = threading.Thread(
            target=read_from_sensor, args=(sensor_ser, controller_ser)
        )
        controller_thread = threading.Thread(
            target=read_from_controller, args=(controller_ser,)
        )
        sensor_thread.start()
        controller_thread.start()

        # Keep the main thread alive while other threads are running
        while sensor_thread.is_alive() or controller_thread.is_alive():
            sensor_thread.join(1)
            controller_thread.join(1)
    finally:
        if sensor_ser and sensor_ser.is_open:
            sensor_ser.close()
            print("Sensor serial connection closed.")
        if controller_ser and controller_ser.is_open:
            controller_ser.close()
            print("Controller serial connection closed.")


if __name__ == "__main__":
    main()
