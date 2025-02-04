import time
import Jetson.GPIO as GPIO

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)  # Use BOARD numbering. Adjust if using BCM mode.

# Define GPIO pins
TRIG_PIN = 18  # Physical pin 18 on Jetson Nano header (for example)
ECHO_PIN = 16  # Physical pin 16 on Jetson Nano header (for example)

# Set up pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    # Ensure trigger is low
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.05)  # 50 ms to stabilize the sensor

    # Trigger the pulse
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure echo time
    start_time = time.time()
    stop_time = time.time()

    # Wait for Echo to go HIGH
    print("Wait for Echo to go HIGH")
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    print("ECHO High")
    # Wait for Echo to go LOW
    print("Wait for Echo to go LOW")
    while GPIO.input(ECHO_PIN) == 1:
        
        stop_time = time.time()
    print("ECHO Low")
    # Calculate pulse length
    elapsed = stop_time - start_time

    # Distance (in cm): speed of sound ~34300 cm/s
    distance = (elapsed * 34300) / 2
    return distance

try:
    # Continuously measure and write to a text file
    while True:
        dist = measure_distance()
        # Print distance (for debugging)
        print("Distance: {:.2f} cm".format(dist))
        
        # Write the distance to a text file
        with open("distance_readings.txt", "w") as f:
            f.write("{:.2f}\n".format(dist))

        time.sleep(0.1)  # Wait 1 second before next measurement

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
