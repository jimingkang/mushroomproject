import serial

# Open the serial port
ser = serial.Serial("/dev/ttyACM0", 115200)

# Create an infinite loop for command input and response handling
while True:
    # Get a command from the user
    cmd = input("Enter a command (or 'exit' to quit): ")

    if cmd.lower() == 'exit':
        break  # Exit the loop if the user enters 'exit'

    # Encode and send the command to the serial port
    ser.write(str.encode(cmd + '\r\n'))

# Close the serial connection when done
ser.close()
