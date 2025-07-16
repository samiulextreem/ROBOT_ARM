import serial
import time
import sys

# ===== SERIAL PORT CONFIGURATION =====
# Change these values to match your actual serial ports
PORT1 = 'COM8'   # First serial port
PORT2 = 'COM12'  # Second serial port
BAUD_RATE = 9600 # Communication speed
TIMEOUT = 1      # Timeout in seconds
# =====================================

def test_serial_ports():
    """
    Test if the configured serial ports can be opened.
    """
    print("=== Testing Serial Ports ===")
    
    # Test PORT1
    try:
        ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=1)
        print(f"✓ {PORT1} - OK (can open and close)")
        ser1.close()
    except serial.SerialException as e:
        print(f"✗ {PORT1} - FAILED: {e}")
    
    # Test PORT2
    try:
        ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=1)
        print(f"✓ {PORT2} - OK (can open and close)")
        ser2.close()
    except serial.SerialException as e:
        print(f"✗ {PORT2} - FAILED: {e}")
    
    print()

def send_integers_to_serial_ports(ser1, ser2):
    """
    Sends two integers to two different serial ports based on user input.
    Ports should already be opened and passed as parameters.
    """
    
    try:
        # Get user input
        print("=== Serial Port Integer Sender ===")
        print(f"Configured ports: {PORT1} and {PORT2}")
        print(f"Baud rate: {BAUD_RATE}")
        print()
        
        # Get integers from user
        try:
            int1 = int(input("Enter first integer to send to " + PORT1 + ": "))
            int2 = int(input("Enter second integer to send to " + PORT2 + ": "))
        except ValueError:
            print("Error: Please enter valid integers only!")
            return
        
        # Send integers
        print("\nSending data...")
        
        # Convert integers to bytes and send
        data1 = str(int1).encode('utf-8') + b'\n'
        data2 = str(int2).encode('utf-8') + b'\n'
        
        print(f"Data to send to {PORT1}: {data1} (raw bytes: {list(data1)})")
        print(f"Data to send to {PORT2}: {data2} (raw bytes: {list(data2)})")
        
        # Send data and check how many bytes were written
        bytes_sent1 = ser1.write(data1)
        print(f"✓ Sent {bytes_sent1} bytes ({int1}) to {PORT1}")
        
        bytes_sent2 = ser2.write(data2)
        print(f"✓ Sent {bytes_sent2} bytes ({int2}) to {PORT2}")
        
        # Flush the output buffers to ensure data is transmitted
        ser1.flush()
        ser2.flush()
        print("✓ Data flushed to both ports")
        
        # Wait a moment for data to be transmitted
        time.sleep(0.5)
        
        # Check if there's any data waiting to be sent
        out_waiting1 = ser1.out_waiting
        out_waiting2 = ser2.out_waiting
        print(f"Bytes still waiting to be sent - {PORT1}: {out_waiting1}, {PORT2}: {out_waiting2}")
        
        print("\n✓ Data sent successfully")
        
    except Exception as e:
        print(f"\nUnexpected error: {e}")

def open_serial_ports():
    """
    Opens and returns both serial port connections.
    """
    print("Opening serial ports...")
    
    try:
        ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ Connected to {PORT1}")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {PORT1}: {e}")
        return None, None
    
    try:
        ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=TIMEOUT)
        print(f"✓ Connected to {PORT2}")
    except serial.SerialException as e:
        print(f"✗ Failed to connect to {PORT2}: {e}")
        ser1.close()
        return None, None
    
    # Wait for serial ports to initialize
    time.sleep(2)
    print("✓ Serial ports ready\n")
    
    return ser1, ser2

if __name__ == "__main__":
    # Check if pyserial is available
    try:
        import serial
    except ImportError:
        print("Error: pyserial library is not installed!")
        print("Install it using: pip install pyserial")
        sys.exit(1)
    
    # Test serial ports first
    test_serial_ports()
    
    # Open serial ports once
    ser1, ser2 = open_serial_ports()
    
    if ser1 is None or ser2 is None:
        print("Failed to open serial ports. Exiting.")
        sys.exit(1)
    
    try:
        # Run in a loop
        print("Serial Port Integer Sender - Press Ctrl+C to exit\n")
        
        while True:
            try:
                send_integers_to_serial_ports(ser1, ser2)
                print("\n" + "-"*50)
                input("Press Enter to send more numbers or Ctrl+C to exit...")
                print()
            except KeyboardInterrupt:
                break
                
    finally:
        # Close ports when exiting
        print("\n\nClosing serial ports...")
        try:
            ser1.close()
            ser2.close()
            print("✓ Serial ports closed")
        except:
            pass
        print("Exiting program. Goodbye!")