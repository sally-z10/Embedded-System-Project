# Automatic Door with Ultrasonic Sensor

## Description
This project is an automatic door control system using an ultrasonic sensor to detect the presence of an object and a servo motor to open or close the door accordingly. The system provides hands-free operation for convenience and safety.

## Components
- **Arduino Uno**: Microcontroller used to control the door movement.
- **Servo Motor**: Used to open and close the door.
- **Ultrasonic Sensor**: Measures the distance to detect the presence of an object.
- **Jumpers**: Used to connect the components.
- **USB Cable**: Provides power supply and uploads the code to the Arduino.

## Modules

### Ultrasonic Sensor
The ultrasonic sensor works by emitting an ultrasonic wave and measuring the time it takes for the echo to return after reflecting off an object.

#### Principle of Operation
1. **Triggering the Pulse**: A 10-microsecond pulse is sent to the TRIG pin. The sensor emits an 8-cycle burst of ultrasonic sound waves at 50 kHz.
2. **Receiving the Echo**: The ECHO pin goes high when the pulse is sent and stays high until the echo is received back.

#### Measuring Distance
The speed of sound in air at room temperature (around 20°C) is approximately 343 m/s. Since the time measured is for the round trip, the distance is calculated by halving this value and dividing the duration in microseconds by approximately 116.7 to get the distance in centimeters.

### Servo Motor
A servo motor is a rotary actuator that allows for precise control of angular position. It consists of a motor coupled with a sensor for position feedback.

#### Principle of Operation
1. **Pulse Width Modulation (PWM)**: The position of the servo motor is determined by the duration of the PWM signal.
   - 1 μs: 0 degrees (minimum position)
   - 1.5 μs: 90 degrees (mid position)
   - 2 μs: 180 degrees (maximum position)
2. **Timer Configuration**: 
   - Timer1 on the Atmega328P is configured for Fast PWM mode.
   - The ICR1 register is set to define a 20ms period (50Hz frequency).
   - The OCR1A register is used to set the pulse width corresponding to the desired angle.

## Circuit & Components
### Connection
- Servo motor connected to PB1.
- Ultrasonic sensor trigger pin connected to PB0.
- Ultrasonic sensor echo pin connected to PB2.
- Ground and Vcc standard as shown in the schematic.

## Code
The code includes necessary libraries and defines the pins used for the servo motor and ultrasonic sensor. It measures the distance using a division factor of 116.62 as calculated previously, initializes both the servo motor and the ultrasonic sensor, and implements the main function logic.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
Special thanks to our mentors and colleagues who helped and supported us throughout this project.

