import simplepyble
import time
import struct

if __name__ == "__main__":
    adapters = simplepyble.Adapter.get_adapters()

    if len(adapters) == 0:
        print("No adapters found")

    # Query the user to pick an adapter
    print("Please select an adapter:")
    for i, adapter in enumerate(adapters):
        print(f"{i}: {adapter.identifier()} [{adapter.address()}]")

    choice = int(input("Enter choice: "))
    adapter = adapters[choice]

    print(f"Selected adapter: {adapter.identifier()} [{adapter.address()}]")

    adapter.set_callback_on_scan_start(lambda: print("Scan started."))
    adapter.set_callback_on_scan_stop(lambda: print("Scan complete."))
    adapter.set_callback_on_scan_found(lambda peripheral: print(f"Found {peripheral.identifier()} [{peripheral.address()}]"))

    # Scan for 5 seconds
    adapter.scan_for(5000)
    peripherals = adapter.scan_get_results()

    # Query the user to pick a peripheral
    print("Please select a peripheral:")
    for i, peripheral in enumerate(peripherals):
        print(f"{i}: {peripheral.identifier()} [{peripheral.address()}]")

    choice = int(input("Enter choice: "))
    peripheral = peripherals[choice]

    print(f"Connecting to: {peripheral.identifier()} [{peripheral.address()}]")
    peripheral.connect()

    print("Successfully connected, listing services...")

    SENSOR_SERVICE_UUID = "c05899c4-457c-4c75-93ab-e55018bb3073"
    DIST_CHAR_UUID    = "c05899c5-457c-4c75-93ab-e55018bb3073"
    GYRO_CHAR_UUID    = "c05899c6-457c-4c75-93ab-e55018bb3073"

    # Write the content to the characteristic
    for i in range(10):
        contents = peripheral.read(SENSOR_SERVICE_UUID, DIST_CHAR_UUID)
        values = struct.unpack('f',contents)
        print(f"Dist: {values[0]}")
        contents = peripheral.read(SENSOR_SERVICE_UUID, GYRO_CHAR_UUID)
        values = struct.unpack('f',contents)
        print(f"Angle: {values[0]}")
        print("==========")
        
        time.sleep(1)

    peripheral.disconnect()

