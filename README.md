# Smart Dustbin

This project implements a robot with obstacle avoidance and hand detection capabilities using Raspberry Pi, GPIO, ultrasonic sensors, and MediaPipe Hands. The robot can detect obstacles, navigate around them, and respond to hand gestures for garbage collection.

## Features

- **Obstacle Avoidance**: Uses an ultrasonic distance sensor to detect obstacles and change direction automatically.
- **Hand Detection**: Detects hand gestures using a webcam and stops the robot for garbage collection when a hand is detected.
- **Motor Control**: Controls the robot's movement with GPIO-connected motors.
- **Integrated Navigation**: Moves forward, stops, and changes direction based on environmental input.

## Requirements

- Raspberry Pi (with GPIO pins)
- Ultrasonic distance sensor (e.g., HC-SR04)
- Two DC motors with a motor driver
- USB Webcam
- Python 3.x
- Libraries: `gpiozero`, `opencv-python`, `mediapipe`

## Installation

1. Clone this repository to your Raspberry Pi:

   ```bash
   git clone https://github.com/yourusername/robot-hand-detection.git
   cd robot-hand-detection
   ```

2. Install the required libraries:

   ```bash
   pip install gpiozero opencv-python mediapipe
   ```

3. Connect the hardware:

   - **Motors**: Connect the motors to GPIO pins 17, 27 (motor1) and 23, 24 (motor2).
   - **Ultrasonic Sensor**: Connect the trigger pin to GPIO 5 and the echo pin to GPIO 6.
   - **Webcam**: Attach a USB webcam to the Raspberry Pi.

4. Ensure proper power supply to the motors and the Raspberry Pi.

## Usage

1. Run the script:

   ```bash
   python robot_control.py
   ```

2. The robot will:

   - Move forward until an obstacle is detected.
   - Stop and reverse for 1 second if an obstacle is within 30 cm.
   - Detect hands using the webcam and stop for 5 seconds to simulate garbage collection.
   - Navigate by turning right or left based on a predefined sequence.

## How It Works

1. **Obstacle Detection**:

   - The ultrasonic sensor measures the distance to obstacles.
   - If an obstacle is detected within 30 cm, the robot reverses and stops.

2. **Hand Detection**:

   - A webcam captures real-time video.
   - MediaPipe Hands detects hand landmarks.
   - If a hand is detected, the robot stops for 5 seconds for garbage collection.

3. **Motor Control**:

   - `gpiozero.Motor` controls the movement of two motors for forward, backward, and turning actions.

## File Structure

- `robot_control.py`: Main script for controlling the robot.
- `README.md`: Project documentation.

## Customization

- **Obstacle Threshold**: Adjust the `ultrasonic_sensor.distance` threshold in meters:
  ```python
  if ultrasonic_sensor.distance < 0.3:
  ```
- **Motor Speed**: Modify the speed of the motors:
  ```python
  motor1.forward(0.5)
  motor2.forward(0.5)
  ```
- **Hand Detection**: Customize the hand detection logic in the `hand_detection()` function.

## Safety Tips

- Ensure the motors are securely connected and powered to prevent damage.
- Test the ultrasonic sensor distance threshold in a safe environment.
- Avoid running the robot near fragile objects.

## Contribution

Feel free to submit issues and pull requests to enhance the project.

## License

This project is licensed under the [MIT License](LICENSE).

