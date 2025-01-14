#define TRIG 0x01          // Trigger pin for ultrasonic sensor
#define ECHO 0x02          // Echo pin for ultrasonic sensor
#define LINE_SENSOR 0x04   // Line sensor input (RB2)
#define LINE_SENSOR2 0x08  // New line sensor input (RB3)

// Motor control pins
#define LEFT_MOTOR_FORWARD 0x01    // RD0
#define LEFT_MOTOR_BACKWARD 0x02   // RD1
#define RIGHT_MOTOR_FORWARD 0x04   // RD2
#define RIGHT_MOTOR_BACKWARD 0x08  // RD3
#define REAR_LEFT_FORWARD 0x10     // RD4
#define REAR_LEFT_BACKWARD 0x20    // RD5
#define REAR_RIGHT_FORWARD 0x40    // RD6
#define REAR_RIGHT_BACKWARD 0x80   // RD7

#define ULTRASONIC_LED 0x10        // RA4 to test ultrasonic detection
#define POTENTIOMETER_CHANNEL 0    // ADC channel for potentiometer

// Function prototypes
void init();
void initPWM();
unsigned int readUltrasonic();
unsigned int readPotentiometer();
void setMotorSpeed(unsigned int speed);
void moveForward();
void moveBackward();
void moveBackwardRight();
void moveBackwardLeft();
void searchAround();
void adel_delay(unsigned int ms);
void adel2_delay(unsigned int ms);
void adel_us_delay(unsigned int us);

// Global flag for line sensor state
volatile unsigned char lineSensorsTriggered = 0;

// Interrupt Service Routine
interrupt() {
    while (!(PORTB & LINE_SENSOR) && !(PORTB & LINE_SENSOR2)) {
        // Both line sensors detect white
        lineSensorsTriggered = 1;
        moveBackward();
    }
    lineSensorsTriggered = 0;
    INTCON &= ~0x02;  // Clear interrupt flag (RBIF)
}

// Main program
void main() {
    unsigned int distance, potValue;

    adel_delay(5000);  // Startup delay
    init();            // Initialize all configurations.
    initPWM();         // Initialize PWM
    INTCON |= 0x90;    // Enable global and RB port change interrupts

    while (1) {
        // If line sensors are triggered, keep moving backward
        if (lineSensorsTriggered) {
            continue;  // Skip the rest of the main loop
        }

        // Ultrasonic distance measurement and motor control
        distance = readUltrasonic();
        potValue = readPotentiometer();

        // Reflect potentiometer value on RC1 (right motor PWM) and RC2 (left motor PWM)
        setMotorSpeed(potValue);

        if (distance > 0 && distance < 30) {
            // Object detected within range, move forward
            PORTA |= ULTRASONIC_LED;
            moveForward();
        } else if (PORTB & LINE_SENSOR) {
            // If Line Sensor 1 (RB2) is high, move backward right
            moveBackwardRight();
        } else if (PORTB & LINE_SENSOR2) {
            // If Line Sensor 2 (RB3) is high, move backward left
            moveBackwardLeft();
        } else {
            // No object detected, search around
            PORTA &= ~ULTRASONIC_LED;
            searchAround();
        }

        adel_delay(100);  // Delay for stability
    }
}

// Initialization function
void init() {
    TRISB = 0x0E;  // Set RB1 (ECHO), RB2 (LINE_SENSOR), and RB3 (LINE_SENSOR2) as inputs
    TRISD = 0x00;  // Set RD0-RD7 as outputs for motor control
    TRISA = 0xFF;  // Set PORTA as input for ADC
    TRISC = 0xF9;  // Set RC1 and RC2 as outputs (PWM pins)

    ADCON0 = 0x41; // Configure ADC: Channel 0, ADC enabled
    ADCON1 = 0x80; // Configure result to be right-justified for 10-bit ADC
    T1CON = 0x00;  // Disable Timer1 initially

}

// PWM Initialization function
void initPWM() {
    // Configure Timer2 for PWM
    T2CON = 0x04;  // Enable Timer2 with a prescaler of 1:1
    PR2 = 0xFF;    // Set the period register for PWM frequency control
    TMR2 = 0x00;   // Clear Timer2

    // Configure CCP1 and CCP2 for PWM output
    CCP1CON = 0x0C;  // PWM mode for CCP1 (RC2)
    CCP2CON = 0x0C;  // PWM mode for CCP2 (RC1)

    // Enable Timer2 to start PWM
    T2CON |= 0x04;  // Set the TMR2ON bit
}

// Function to set motor speed using PWM
void setMotorSpeed(unsigned int speed) {
    unsigned char pwm_value = speed / 4;  // Scale 10-bit ADC to 8-bit PWM (0-255)

    CCPR1L = pwm_value;  // Set duty cycle for left motor (RC2)
    CCPR2L = pwm_value;  // Set duty cycle for right motor (RC1)
}

// Function to read potentiometer value
unsigned int readPotentiometer() {
    ADCON0 &= 0xC7;                     // Select Channel 0 (POTENTIOMETER_CHANNEL)
    ADCON0 |= (POTENTIOMETER_CHANNEL << 3); // Set ADC channel
    adel_us_delay(20);                  // Acquisition time
    ADCON0 |= 0x04;                     // Set the GO/DONE bit to start conversion
    while (ADCON0 & 0x04);              // Wait for the GO/DONE bit to clear
    return ((ADRESH << 8) | ADRESL);    // Combine ADRESH and ADRESL for 10-bit ADC result
}

// Function to move the robot forward
void moveForward() {
    PORTD = LEFT_MOTOR_FORWARD | RIGHT_MOTOR_FORWARD | REAR_LEFT_FORWARD | REAR_RIGHT_FORWARD;
}

// Function to move the robot backward
void moveBackward() {
    PORTD = LEFT_MOTOR_BACKWARD | RIGHT_MOTOR_BACKWARD | REAR_LEFT_BACKWARD | REAR_RIGHT_BACKWARD;
    adel2_delay(500);
}

// Function to move the robot backward right
void moveBackwardRight() {
    PORTD = LEFT_MOTOR_BACKWARD | REAR_LEFT_BACKWARD | RIGHT_MOTOR_FORWARD | REAR_RIGHT_FORWARD;
    adel_delay(500);
}

// Function to move the robot backward left
void moveBackwardLeft() {
    PORTD = RIGHT_MOTOR_BACKWARD | REAR_RIGHT_BACKWARD | LEFT_MOTOR_FORWARD | REAR_LEFT_FORWARD;
    adel_delay(500);
}

// Function to search around (rotate)
void searchAround() {
    PORTD = LEFT_MOTOR_FORWARD | REAR_LEFT_FORWARD | RIGHT_MOTOR_BACKWARD | REAR_RIGHT_BACKWARD;
    adel_delay(500);
}

// Function to read ultrasonic distance
unsigned int readUltrasonic() {
    unsigned int time = 0;
    unsigned int timeout;

    PORTB |= TRIG;
    adel_us_delay(10);
    PORTB &= ~TRIG;

    timeout = 0xFFFF;
    while (!(PORTB & ECHO)) {
        if (--timeout == 0) return 0;
    }

    TMR1H = 0; TMR1L = 0;
    T1CON = 0x01;

    timeout = 0xFFFF;
    while (PORTB & ECHO) {
        if (--timeout == 0) {
            T1CON = 0x00;
            return 0;
        }
    }

    T1CON = 0x00;
    time = (TMR1H << 8) | TMR1L;

    return time / 58;
}

// Delay functions
void adel_delay(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 111; j++) NOP();
    }
}
void adel2_delay(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 111; j++) NOP();
    }
}

void adel_us_delay(unsigned int us) {
    unsigned int i;
    for (i = 0; i < us; i++) NOP(); NOP(); NOP(); NOP();
}