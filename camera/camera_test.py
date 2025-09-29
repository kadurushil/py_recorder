# simple_camera_test.py
import cv2
import time

# --- Camera Settings ---
DEVICE_INDEX = 0
REQUESTED_WIDTH = 1920
REQUESTED_HEIGHT = 1080
REQUESTED_FPS = 30
# ---------------------

cap = cv2.VideoCapture(DEVICE_INDEX)
if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

# --- Apply Settings ---
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUESTED_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUESTED_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, REQUESTED_FPS)
# ---------------------

print("Camera opened. Press 'q' to quit.")
print(f"Driver reported FPS: {cap.get(cv2.CAP_PROP_FPS)}")

frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break

    frame_count += 1

    # Calculate and display FPS every 30 frames
    if frame_count % 30 == 0:
        end_time = time.time()
        elapsed = end_time - start_time
        current_fps = frame_count / elapsed
        print(f"Measured FPS: {current_fps:.2f}")

    # Display the resulting frame (optional, can be commented out)
    cv2.imshow('Camera Test', frame)

    if cv2.waitKey(1) == ord('q'):
        break

# --- Cleanup ---
end_time = time.time()
total_elapsed = end_time - start_time
average_fps = frame_count / total_elapsed
print("\n--- Results ---")
print(f"Total frames captured: {frame_count}")
print(f"Total time: {total_elapsed:.2f} seconds")
print(f"Average Measured FPS: {average_fps:.2f}")
print("---------------")

cap.release()
cv2.destroyAllWindows()