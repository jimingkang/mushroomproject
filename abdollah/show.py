import cv2
import numpy as np
import pyrealsense2 as rs
import ctypes
from ultralytics import YOLO
import queue
from concurrent.futures import ThreadPoolExecutor, as_completed
from itertools import cycle

# Initialize queues
input_queue = queue.Queue()
output_queue = queue.Queue()

# Load YOLOv8 models in separate instances
def load_model():
    return YOLO("yolov8n.engine", task="segment")

# Number of model instances
num_models = 2

# Create a pool of YOLO models
model_pool = [load_model() for _ in range(num_models)]

# Prediction function to be run by the workers
def process_image(data):
    color_image, model = data
    result = model.predict(color_image, imgsz=(736, 1280))
    return result[0].plot()  # Assuming result[0].plot() returns the image with predictions drawn

def worker(input_queue, output_queue, model_pool):
    with ThreadPoolExecutor(max_workers=len(model_pool)) as executor:
        model_cycle = cycle(model_pool)  # Create a cycle of models

        while True:
            if input_queue.empty():
                continue  # Wait until the queue has an item

            color_image = input_queue.get()  # Fetch the next image to process
            if color_image is None:
                break  # Stop if None is sent as a signal to terminate

            # Get the next model from the cycle
            model = next(model_cycle)
            
            # Submit the task to the executor
            future = executor.submit(process_image, (color_image, model))
            
            # Process the result as soon as it's ready
            result = future.result()
            output_queue.put(result)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Main execution flow in the try block
try:
    # Start the worker thread
    from threading import Thread
    worker_thread = Thread(target=worker, args=(input_queue, output_queue, model_pool))
    worker_thread.start()

    while True:
        # Wait for a new frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert RealSense frame to OpenCV format and add to the input queue
        color_image = np.asanyarray(color_frame.get_data())
        input_queue.put(color_image)

        # Display results if available in the output queue
        if not output_queue.empty():
            result = output_queue.get()  # Get the next processed image
            cv2.imshow("Object Detection", result)

        # Check for the 'q' key to quit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    input_queue.put(None)
    worker_thread.join()
    pipeline.stop()
    cv2.destroyAllWindows()