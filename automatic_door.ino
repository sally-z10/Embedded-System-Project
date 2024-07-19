
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_PIN PB0   // Ultrasonic sensor trigger pin connected to PD6 (digital pin 8)
#define SERVO_PIN PB1  // Servo motor connected to PB1 (OC1A, digital pin 9)
#define ECHO_PIN PB2   // Ultrasonic sensor echo pin connected to PD7 (digital pin 10)

void initServo() {
    // Set the Servo pin as output
    DDRB |= (1 << SERVO_PIN);

    // Configure Timer1 for Fast PWM mode, 8-bit
    TCCR1A |= (1 << WGM11) | (1 << COM1A1);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler = 8

    // Set ICR1 to define the top value for the 20ms period (50Hz PWM)
    ICR1 = 39999;  // ICR1 = 16 MHz / (8 * 50 Hz) - 1

    // Start with the servo at 0 degrees
    OCR1A = 999;  // 1.5 ms pulse width for 0 degrees
}

void setServoPosition(uint8_t angle) {
    // Map the angle (0-180) to pulse width (1ms - 2ms)
    uint16_t pulseWidth = 999 + (angle * 22.22);

    // Set the pulse width to move the servo
    OCR1A = pulseWidth;
}

void initUltrasonicSensor() {
    // Set the TRIG_PIN as output and ECHO_PIN as input
    DDRB |= (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
}

long measureDistance() {
    // Send a trigger pulse to the ultrasonic sensor
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    // Measure the duration of the echo pulse
    float duration = 0;
    TCNT1 = 0;  // Reset Timer1
    while (!(PINB & (1 << ECHO_PIN))) {  // Wait for the echo to start
        // if (TCNT1 > 60000) {  // Timeout after ~30ms
        //     duration = 0;
        //     break;
        // }
    }
    TCNT1 = 0;  // Reset Timer1
    while (PINB & (1 << ECHO_PIN)) {  // Wait for the echo to end
        // if (TCNT1 > 60000) {  // Timeout after ~30ms
        //     duration = 0;
        //     break;
        // }
        duration = TCNT1;
    }

    // Convert the duration to distance in cm speed
    float distance = (duration / 116.62); 

    return distance;
}

int main(void) {
    // Initialize the servo, ultrasonic sensor, and UART
    initServo();
    initUltrasonicSensor(); 

    while (1) {
        // Measure the distance
        float distance = measureDistance();

        // Control the servo based on distance
        if (distance > 0 && distance < 30) {
            setServoPosition(90);  // Open position
        } else {
            setServoPosition(0);  // Closed position
        }

        _delay_ms(1000);  // Wait for 1 second before the next measurement
    }
}
/////////////////////////////////////////////////////////////////////
