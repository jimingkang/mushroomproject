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

# import serial
# import time

# port_name = "/dev/ttyACM0"
# baud_rate = 115200
# command = 'G91 G21 G1 X1 Y1\n'  # Make sure to include '\n' to send the command properly

# # Call the function to send the command and get the response

# ser = serial.Serial(port_name, baud_rate, timeout=None)

# def test():
#     ser.write(command.encode())
#     input()
#     print("ok")

# # Specify your serial port and command
# test()

# """
# This is a simple script that attempts to connect to the GRBL controller at 
# > /dev/tty.usbserial-A906L14X
# It then reads the grbl_test.gcode and sends it to the controller

# The script waits for the completion of the sent line of gcode before moving onto the next line

# tested on
# > MacOs Monterey arm64
# > Python 3.9.5 | packaged by conda-forge | (default, Jun 19 2021, 00:24:55) 
# [Clang 11.1.0 ] on darwin
# > Vscode 1.62.3
# > Openbuilds BlackBox GRBL controller
# > GRBL 1.1

# """

# import serial
# import time
# from threading import Event

# BAUD_RATE = 115200

# def remove_comment(string):
#     if (string.find(';') == -1):
#         return string
#     else:
#         return string[:string.index(';')]


# def remove_eol_chars(string):
#     # removed \n or traling spaces
#     return string.strip()


# def send_wake_up(ser):
#     # Wake up
#     # Hit enter a few times to wake the Printrbot
#     ser.write(str.encode("\r\n\r\n"))
#     time.sleep(2)   # Wait for Printrbot to initialize
#     ser.flushInput()  # Flush startup text in serial input

# def wait_for_movement_completion(ser,cleaned_line):

#     Event().wait(1)

#     if cleaned_line != '$X' or '$$':

#         idle_counter = 0

#         while True:

#             # Event().wait(0.01)
#             ser.reset_input_buffer()
#             command = str.encode('?' + '\n')
#             ser.write(command)
#             grbl_out = ser.readline() 
#             grbl_response = grbl_out.strip().decode('utf-8')

#             if grbl_response != 'ok':

#                 if grbl_response.find('Idle') > 0:
#                     idle_counter += 1

#             if idle_counter > 10:
#                 break
#     return


# def stream_gcode(GRBL_port_path,gcode_path):
#     # with contect opens file/connection and closes it if function(with) scope is left
#     with open(gcode_path, "r") as file, serial.Serial(GRBL_port_path, BAUD_RATE) as ser:
#         send_wake_up(ser)
#         for line in file:
#             # cleaning up gcode from file
#             cleaned_line = remove_eol_chars(remove_comment(line))
#             if cleaned_line:  # checks if string is empty
#                 print("Sending gcode:" + str(cleaned_line))
#                 # converts string to byte encoded string and append newline
#                 command = str.encode(line + '\n')
#                 ser.write(command)  # Send g-code

#                 wait_for_movement_completion(ser,cleaned_line)

#                 grbl_out = ser.readline()  # Wait for response with carriage return
#                 print(" : " , grbl_out.strip().decode('utf-8'))

                
        
#         print('End of gcode')

# if __name__ == "__main__":

#     # GRBL_port_path = '/dev/tty.usbserial-A906L14X'
#     GRBL_port_path = '/dev/ttyACM0'
#     gcode_path = 'grbl_test.gcode'

#     print("USB Port: ", GRBL_port_path)
#     print("Gcode file: ", gcode_path)
#     stream_gcode(GRBL_port_path,gcode_path)

#     print('EOF')