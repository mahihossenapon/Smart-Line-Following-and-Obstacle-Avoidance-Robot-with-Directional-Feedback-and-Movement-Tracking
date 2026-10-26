# Smart Line Following and Obstacle Avoidance Robot with Directional Feedback and Movement Tracking

An Arduino-based autonomous mobile robot that follows a black line, detects obstacles, navigates around them, and attempts to return to its original path. The system combines infrared sensing, ultrasonic distance measurement, and differential motor control for autonomous navigation.

## Features

- **Line Following:** Tracks a black line on a light-colored surface using an 8-channel IR sensor array.
- **Obstacle Detection:** Uses two HC-SR04 ultrasonic sensors to detect obstacles ahead and monitor side clearance.
- **Obstacle Avoidance:** Executes a right-turn-based maneuver to pass obstacles and search for the line again.
- **Directional Control:** Adjusts left and right motor speeds to control forward movement and turning.
- **Movement Monitoring:** Provides serial feedback on obstacle distances, line recovery, and navigation status.
- **Automatic Navigation:** Switches between line-following and obstacle-avoidance operations without manual steering.

## Hardware Components

| Component | Quantity |
|---|---:|
| Arduino Nano | 1 |
| L298N Motor Driver | 1 |
| Custom 8-channel IR Sensor Array | 1 |
| 74HC4067 Multiplexer | 1 |
| HC-SR04 Ultrasonic Sensors | 2 |
| 16 GA Motors | 2 |
| 3D-Printed Wheels | 2 |
| 1600 mAh Li-Po Battery | 1 |
| LM2596 Buck Converter | 1 |
| XL6009 Boost Converter | 1 |

## Working Principle

1. The IR sensor array detects the black line by measuring differences in reflected infrared light.
2. The Arduino processes the sensor readings and adjusts motor speeds to keep the robot aligned with the path.
3. When the front ultrasonic sensor detects an obstacle within the configured threshold of 12 cm, the robot initiates obstacle avoidance.
4. The robot turns right and moves alongside the obstacle while monitoring the front and left-side distances.
5. After passing the obstacle, it performs a sequence of turns and searches for the black line again.
6. Once the line is detected, normal line-following resumes.

## Repository Structure

```text
Smart-Line-Following-and-Obstacle-Avoidance-Robot/
├── main_code.ino
├── Circuit_Diagram.png
├── Line Follower Robot (Top View).jpg
├── Pin_Connection_Summary.png
└── README.md
```

## Getting Started

1. Clone or download this repository.
2. Open `main_code.ino` in the Arduino IDE.
3. Assemble the circuit according to the circuit diagram and pin connection summary.
4. Connect the Arduino Nano to your computer and select the correct board and port.
5. Upload the code and place the robot on a black line over a light-colored surface.
6. Open the Serial Monitor at **9600 baud** to observe navigation and sensor feedback.

**Note:** Calibrate the IR sensor reference values, motor speeds, and obstacle thresholds according to your hardware and operating surface.

## Limitations

Sensor readings may vary with lighting, surface reflectivity, and line contrast. Motor imbalance and fixed turning delays may also affect obstacle avoidance and line recovery.

## Future Improvements

- Implement more robust PID-based line-following control.
- Improve obstacle avoidance using adaptive turning and distance control.
- Add wheel encoders for quantitative movement and distance tracking.
- Improve sensor calibration and recovery when the line is lost.

## Project Information

**Platform:** Arduino Nano  
**Programming Language:** C/C++ (Arduino)  
**Application:** Autonomous Robotics and Embedded Systems

This project was developed as part of an academic laboratory experiment in autonomous robot navigation.
