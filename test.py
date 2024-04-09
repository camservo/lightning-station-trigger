import asyncio
from bleak import BleakClient

# The address of your device (replace with the actual address of your BLE device)
DEVICE_ADDRESS = "30182F81-626D-7E6C-760B-CCCC3B14F2AB"

# UUIDs for the service and characteristic (same as in the Arduino example)
SERVICE_UUID = "12345678-1234-1234-1234-123456789ABC"
CHARACTERISTIC_UUID = "87654321-4321-4321-4321-CBA987654321"

async def run_ble_client(device_address: str):
    async with BleakClient(device_address) as client:
        # Connect to the device
        connected = await client.connect()
        if connected:
            print(f"Connected to {device_address}")

            # Function to handle incoming notifications
            def notification_handler(sender, data):
                print("Received data:", data.decode())

            # Subscribe to the characteristic
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            print("Subscribed to characteristic updates. Listening for data...")
            # Keep the program running to listen for notifications
            await asyncio.sleep(30)  # Listen for 30 seconds, adjust as needed

            # Unsubscribe from notifications and disconnect
            await client.stop_notify(CHARACTERISTIC_UUID)
            await client.disconnect()
        else:
            print("Failed to connect")

# Replace DEVICE_ADDRESS with your BLE device address
loop = asyncio.get_event_loop()
loop.run_until_complete(run_ble_client(DEVICE_ADDRESS))