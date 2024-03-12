from gpiozero import Motor, DistanceSensor
from time import sleep
import cv2
import mediapipe as mp

# Assuming the motors are connected to GPIO pins 17, 18, 22, and 23
motor1 = Motor(forward=17, backward=27)
motor2 = Motor(forward=23, backward=24)

# Assuming the ultrasonic sensor is connected to GPIO pins 24 (trigger) and 25 (echo)
ultrasonic_sensor = DistanceSensor(echo=6, trigger=5)

def hand_detection():
    # Initialize MediaPipe Hands
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands()

    # Open webcam
    cap = cv2.VideoCapture(0)

    # Initialize drawing module
    mp_drawing = mp.solutions.drawing_utils

    while cap.isOpened():
        # Read frame from webcam
        ret, frame = cap.read()
        if not ret:
            continue

        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Hands
        results = hands.process(rgb_frame)

        # If hands are detected, return True
        if results.multi_hand_landmarks:
            return True

        # Display the frame
        cv2.imshow('Hand Detection', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close the window
    cap.release()
    cv2.destroyAllWindows()

    # Return False if no hands are detected
    return False

try:
    while True:
        # Check for obstacles
        if ultrasonic_sensor.distance < 0.3:  # Adjust the threshold distance as needed
            print("Obstacle detected! Changing direction.")
            motor1.backward(0.5)
            motor2.backward(0.5)
            sleep(1)  # Reverse for 1 second
            motor1.stop()
            motor2.stop()
            continue  # Skip the rest of the loop and start again

        # Check for hand detection
        if hand_detection():
            print("Hand detected! Stopping motors for garbage collection.")
            motor1.stop()
            motor2.stop()
            # Perform garbage collection (stop the robot, collect garbage, etc.)
            sleep(5)  # Simulating the garbage collection process
            print("Resuming robot movement.")
        else:
            print("No hand detected. Continuing robot movement.")
            motor1.forward(0.5)
            motor2.forward(0.5)
            sleep(5)  # Move forward for 5 seconds

        motor1.stop()
        motor2.stop()
        sleep(1)  # Stop for 1 second

        # Turn right
        motor1.forward(0.5)
        motor2.backward(0.5)
        sleep(2)  # Turn right for 2 seconds

        motor1.stop()
        motor2.stop()
        sleep(1)  # Stop for 1 second

        # Turn left
        motor1.backward(0.5)
        motor2.forward(0.5)
        sleep(2)  # Turn left for 2 seconds

        motor1.stop()
        motor2.stop()
        sleep(1)  # Stop for 1 second
except KeyboardInterrupt:
    print("Stopping motors and exiting.")
    motor1.stop()
    motor2.stop()
