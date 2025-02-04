import threading
import time
import Jetson.GPIO as GPIO
from hitbot.HitbotInterface import HitbotInterface

# --------------------
# Robot Initialization
# --------------------
hi = HitbotInterface(92)  # Use your robot ID
hi.net_port_initial()
ret = hi.initial(1, 210)
print("Connected:", hi.is_connect())
print("Unlock Position:", hi.unlock_position())

stop_event = threading.Event()

# --------------------
# Ultrasonic Setup
# --------------------
GPIO.setmode(GPIO.BOARD)

TRIG_PIN = 18  # Physical pin 18 (example)
ECHO_PIN = 16  # Physical pin 16 (example)

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    # Ensure trigger is low
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.05)  # let sensor stabilize

    # Trigger the pulse
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure echo time
    start_time = time.time()
    stop_time = time.time()

    # Wait for echo start
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    # Wait for echo end
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2
    return distance

def sensor_monitor():
    # Continuously measure distance and check if less than 5cm
    while not stop_event.is_set():
        dist = measure_distance()
        # If distance < 5cm, trigger emergency stop
        if dist < 20:
            print("Emergency stop triggered due to close distance!")
            hi.stop_move()
            stop_event.set()
        time.sleep(0.1)

# --------------------
# Movement Loop
# --------------------
def move_loop():
    try:
        while not stop_event.is_set():
            # Move to first angle
            hi.movej_angle(0, 0, 0, -60, 100, 0)
            hi.wait_stop()
            if stop_event.is_set():
                break

            # Move to second angle
            hi.movej_angle(0, 90, 0, -60, 100, 0)
            hi.wait_stop()
    except KeyboardInterrupt:
        print("Movement interrupted by user.")
    finally:
        print("Movement loop ending...")

# --------------------
# Start Threads
# --------------------
sensor_thread = threading.Thread(target=sensor_monitor, daemon=True)
sensor_thread.start()

move_thread = threading.Thread(target=move_loop)
move_thread.start()

# Wait until movement thread finishes
move_thread.join()

# Clean up GPIO
GPIO.cleanup()

print("All threads have stopped. Program exiting.")
