
_interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;MyProject.c,38 :: 		interrupt() {
;MyProject.c,39 :: 		while (!(PORTB & LINE_SENSOR) && !(PORTB & LINE_SENSOR2)) {
L_interrupt0:
	BTFSC      PORTB+0, 2
	GOTO       L_interrupt1
	BTFSC      PORTB+0, 3
	GOTO       L_interrupt1
L__interrupt38:
;MyProject.c,41 :: 		lineSensorsTriggered = 1;
	MOVLW      1
	MOVWF      _lineSensorsTriggered+0
;MyProject.c,42 :: 		moveBackward();
	CALL       _moveBackward+0
;MyProject.c,43 :: 		}
	GOTO       L_interrupt0
L_interrupt1:
;MyProject.c,44 :: 		lineSensorsTriggered = 0;
	CLRF       _lineSensorsTriggered+0
;MyProject.c,45 :: 		INTCON &= ~0x02;  // Clear interrupt flag (RBIF)
	BCF        INTCON+0, 1
;MyProject.c,46 :: 		}
L_end_interrupt:
L__interrupt41:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_main:

;MyProject.c,49 :: 		void main() {
;MyProject.c,52 :: 		adel_delay(5000);  // Startup delay
	MOVLW      136
	MOVWF      FARG_adel_delay_ms+0
	MOVLW      19
	MOVWF      FARG_adel_delay_ms+1
	CALL       _adel_delay+0
;MyProject.c,53 :: 		init();            // Initialize all configurations.
	CALL       _init+0
;MyProject.c,54 :: 		initPWM();         // Initialize PWM
	CALL       _initPWM+0
;MyProject.c,55 :: 		INTCON |= 0x90;    // Enable global and RB port change interrupts
	MOVLW      144
	IORWF      INTCON+0, 1
;MyProject.c,57 :: 		while (1) {
L_main4:
;MyProject.c,59 :: 		if (lineSensorsTriggered) {
	MOVF       _lineSensorsTriggered+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main6
;MyProject.c,60 :: 		continue;  // Skip the rest of the main loop
	GOTO       L_main4
;MyProject.c,61 :: 		}
L_main6:
;MyProject.c,64 :: 		distance = readUltrasonic();
	CALL       _readUltrasonic+0
	MOVF       R0+0, 0
	MOVWF      main_distance_L0+0
	MOVF       R0+1, 0
	MOVWF      main_distance_L0+1
;MyProject.c,65 :: 		potValue = readPotentiometer();
	CALL       _readPotentiometer+0
;MyProject.c,68 :: 		setMotorSpeed(potValue);
	MOVF       R0+0, 0
	MOVWF      FARG_setMotorSpeed_speed+0
	MOVF       R0+1, 0
	MOVWF      FARG_setMotorSpeed_speed+1
	CALL       _setMotorSpeed+0
;MyProject.c,70 :: 		if (distance > 0 && distance < 30) {
	MOVF       main_distance_L0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main43
	MOVF       main_distance_L0+0, 0
	SUBLW      0
L__main43:
	BTFSC      STATUS+0, 0
	GOTO       L_main9
	MOVLW      0
	SUBWF      main_distance_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main44
	MOVLW      30
	SUBWF      main_distance_L0+0, 0
L__main44:
	BTFSC      STATUS+0, 0
	GOTO       L_main9
L__main39:
;MyProject.c,72 :: 		PORTA |= ULTRASONIC_LED;
	BSF        PORTA+0, 4
;MyProject.c,73 :: 		moveForward();
	CALL       _moveForward+0
;MyProject.c,74 :: 		} else if (PORTB & LINE_SENSOR) {
	GOTO       L_main10
L_main9:
	BTFSS      PORTB+0, 2
	GOTO       L_main11
;MyProject.c,76 :: 		moveBackwardRight();
	CALL       _moveBackwardRight+0
;MyProject.c,77 :: 		} else if (PORTB & LINE_SENSOR2) {
	GOTO       L_main12
L_main11:
	BTFSS      PORTB+0, 3
	GOTO       L_main13
;MyProject.c,79 :: 		moveBackwardLeft();
	CALL       _moveBackwardLeft+0
;MyProject.c,80 :: 		} else {
	GOTO       L_main14
L_main13:
;MyProject.c,82 :: 		PORTA &= ~ULTRASONIC_LED;
	BCF        PORTA+0, 4
;MyProject.c,83 :: 		searchAround();
	CALL       _searchAround+0
;MyProject.c,84 :: 		}
L_main14:
L_main12:
L_main10:
;MyProject.c,86 :: 		adel_delay(100);  // Delay for stability
	MOVLW      100
	MOVWF      FARG_adel_delay_ms+0
	MOVLW      0
	MOVWF      FARG_adel_delay_ms+1
	CALL       _adel_delay+0
;MyProject.c,87 :: 		}
	GOTO       L_main4
;MyProject.c,88 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_init:

;MyProject.c,91 :: 		void init() {
;MyProject.c,92 :: 		TRISB = 0x0E;  // Set RB1 (ECHO), RB2 (LINE_SENSOR), and RB3 (LINE_SENSOR2) as inputs
	MOVLW      14
	MOVWF      TRISB+0
;MyProject.c,93 :: 		TRISD = 0x00;  // Set RD0-RD7 as outputs for motor control
	CLRF       TRISD+0
;MyProject.c,94 :: 		TRISA = 0xFF;  // Set PORTA as input for ADC
	MOVLW      255
	MOVWF      TRISA+0
;MyProject.c,95 :: 		TRISC = 0xF9;  // Set RC1 and RC2 as outputs (PWM pins)
	MOVLW      249
	MOVWF      TRISC+0
;MyProject.c,97 :: 		ADCON0 = 0x41; // Configure ADC: Channel 0, ADC enabled
	MOVLW      65
	MOVWF      ADCON0+0
;MyProject.c,98 :: 		ADCON1 = 0x80; // Configure result to be right-justified for 10-bit ADC
	MOVLW      128
	MOVWF      ADCON1+0
;MyProject.c,99 :: 		T1CON = 0x00;  // Disable Timer1 initially
	CLRF       T1CON+0
;MyProject.c,101 :: 		}
L_end_init:
	RETURN
; end of _init

_initPWM:

;MyProject.c,104 :: 		void initPWM() {
;MyProject.c,106 :: 		T2CON = 0x04;  // Enable Timer2 with a prescaler of 1:1
	MOVLW      4
	MOVWF      T2CON+0
;MyProject.c,107 :: 		PR2 = 0xFF;    // Set the period register for PWM frequency control
	MOVLW      255
	MOVWF      PR2+0
;MyProject.c,108 :: 		TMR2 = 0x00;   // Clear Timer2
	CLRF       TMR2+0
;MyProject.c,111 :: 		CCP1CON = 0x0C;  // PWM mode for CCP1 (RC2)
	MOVLW      12
	MOVWF      CCP1CON+0
;MyProject.c,112 :: 		CCP2CON = 0x0C;  // PWM mode for CCP2 (RC1)
	MOVLW      12
	MOVWF      CCP2CON+0
;MyProject.c,115 :: 		T2CON |= 0x04;  // Set the TMR2ON bit
	BSF        T2CON+0, 2
;MyProject.c,116 :: 		}
L_end_initPWM:
	RETURN
; end of _initPWM

_setMotorSpeed:

;MyProject.c,119 :: 		void setMotorSpeed(unsigned int speed) {
;MyProject.c,120 :: 		unsigned char pwm_value = speed / 4;  // Scale 10-bit ADC to 8-bit PWM (0-255)
	MOVF       FARG_setMotorSpeed_speed+0, 0
	MOVWF      R0+0
	MOVF       FARG_setMotorSpeed_speed+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
;MyProject.c,122 :: 		CCPR1L = pwm_value;  // Set duty cycle for left motor (RC2)
	MOVF       R0+0, 0
	MOVWF      CCPR1L+0
;MyProject.c,123 :: 		CCPR2L = pwm_value;  // Set duty cycle for right motor (RC1)
	MOVF       R0+0, 0
	MOVWF      CCPR2L+0
;MyProject.c,124 :: 		}
L_end_setMotorSpeed:
	RETURN
; end of _setMotorSpeed

_readPotentiometer:

;MyProject.c,127 :: 		unsigned int readPotentiometer() {
;MyProject.c,128 :: 		ADCON0 &= 0xC7;                     // Select Channel 0 (POTENTIOMETER_CHANNEL)
	MOVLW      199
	ANDWF      ADCON0+0, 1
;MyProject.c,129 :: 		ADCON0 |= (POTENTIOMETER_CHANNEL << 3); // Set ADC channel
;MyProject.c,130 :: 		adel_us_delay(20);                  // Acquisition time
	MOVLW      20
	MOVWF      FARG_adel_us_delay_us+0
	MOVLW      0
	MOVWF      FARG_adel_us_delay_us+1
	CALL       _adel_us_delay+0
;MyProject.c,131 :: 		ADCON0 |= 0x04;                     // Set the GO/DONE bit to start conversion
	BSF        ADCON0+0, 2
;MyProject.c,132 :: 		while (ADCON0 & 0x04);              // Wait for the GO/DONE bit to clear
L_readPotentiometer15:
	BTFSS      ADCON0+0, 2
	GOTO       L_readPotentiometer16
	GOTO       L_readPotentiometer15
L_readPotentiometer16:
;MyProject.c,133 :: 		return ((ADRESH << 8) | ADRESL);    // Combine ADRESH and ADRESL for 10-bit ADC result
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;MyProject.c,134 :: 		}
L_end_readPotentiometer:
	RETURN
; end of _readPotentiometer

_moveForward:

;MyProject.c,137 :: 		void moveForward() {
;MyProject.c,138 :: 		PORTD = LEFT_MOTOR_FORWARD | RIGHT_MOTOR_FORWARD | REAR_LEFT_FORWARD | REAR_RIGHT_FORWARD;
	MOVLW      85
	MOVWF      PORTD+0
;MyProject.c,139 :: 		}
L_end_moveForward:
	RETURN
; end of _moveForward

_moveBackward:

;MyProject.c,142 :: 		void moveBackward() {
;MyProject.c,143 :: 		PORTD = LEFT_MOTOR_BACKWARD | RIGHT_MOTOR_BACKWARD | REAR_LEFT_BACKWARD | REAR_RIGHT_BACKWARD;
	MOVLW      170
	MOVWF      PORTD+0
;MyProject.c,144 :: 		adel2_delay(500);
	MOVLW      244
	MOVWF      FARG_adel2_delay_ms+0
	MOVLW      1
	MOVWF      FARG_adel2_delay_ms+1
	CALL       _adel2_delay+0
;MyProject.c,145 :: 		}
L_end_moveBackward:
	RETURN
; end of _moveBackward

_moveBackwardRight:

;MyProject.c,148 :: 		void moveBackwardRight() {
;MyProject.c,149 :: 		PORTD = LEFT_MOTOR_BACKWARD | REAR_LEFT_BACKWARD | RIGHT_MOTOR_FORWARD | REAR_RIGHT_FORWARD;
	MOVLW      102
	MOVWF      PORTD+0
;MyProject.c,150 :: 		adel_delay(500);
	MOVLW      244
	MOVWF      FARG_adel_delay_ms+0
	MOVLW      1
	MOVWF      FARG_adel_delay_ms+1
	CALL       _adel_delay+0
;MyProject.c,151 :: 		}
L_end_moveBackwardRight:
	RETURN
; end of _moveBackwardRight

_moveBackwardLeft:

;MyProject.c,154 :: 		void moveBackwardLeft() {
;MyProject.c,155 :: 		PORTD = RIGHT_MOTOR_BACKWARD | REAR_RIGHT_BACKWARD | LEFT_MOTOR_FORWARD | REAR_LEFT_FORWARD;
	MOVLW      153
	MOVWF      PORTD+0
;MyProject.c,156 :: 		adel_delay(500);
	MOVLW      244
	MOVWF      FARG_adel_delay_ms+0
	MOVLW      1
	MOVWF      FARG_adel_delay_ms+1
	CALL       _adel_delay+0
;MyProject.c,157 :: 		}
L_end_moveBackwardLeft:
	RETURN
; end of _moveBackwardLeft

_searchAround:

;MyProject.c,160 :: 		void searchAround() {
;MyProject.c,161 :: 		PORTD = LEFT_MOTOR_FORWARD | REAR_LEFT_FORWARD | RIGHT_MOTOR_BACKWARD | REAR_RIGHT_BACKWARD;
	MOVLW      153
	MOVWF      PORTD+0
;MyProject.c,162 :: 		adel_delay(500);
	MOVLW      244
	MOVWF      FARG_adel_delay_ms+0
	MOVLW      1
	MOVWF      FARG_adel_delay_ms+1
	CALL       _adel_delay+0
;MyProject.c,163 :: 		}
L_end_searchAround:
	RETURN
; end of _searchAround

_readUltrasonic:

;MyProject.c,166 :: 		unsigned int readUltrasonic() {
;MyProject.c,167 :: 		unsigned int time = 0;
;MyProject.c,170 :: 		PORTB |= TRIG;
	BSF        PORTB+0, 0
;MyProject.c,171 :: 		adel_us_delay(10);
	MOVLW      10
	MOVWF      FARG_adel_us_delay_us+0
	MOVLW      0
	MOVWF      FARG_adel_us_delay_us+1
	CALL       _adel_us_delay+0
;MyProject.c,172 :: 		PORTB &= ~TRIG;
	BCF        PORTB+0, 0
;MyProject.c,174 :: 		timeout = 0xFFFF;
	MOVLW      255
	MOVWF      readUltrasonic_timeout_L0+0
	MOVLW      255
	MOVWF      readUltrasonic_timeout_L0+1
;MyProject.c,175 :: 		while (!(PORTB & ECHO)) {
L_readUltrasonic17:
	BTFSC      PORTB+0, 1
	GOTO       L_readUltrasonic18
;MyProject.c,176 :: 		if (--timeout == 0) return 0;
	MOVLW      1
	SUBWF      readUltrasonic_timeout_L0+0, 1
	BTFSS      STATUS+0, 0
	DECF       readUltrasonic_timeout_L0+1, 1
	MOVLW      0
	XORWF      readUltrasonic_timeout_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__readUltrasonic55
	MOVLW      0
	XORWF      readUltrasonic_timeout_L0+0, 0
L__readUltrasonic55:
	BTFSS      STATUS+0, 2
	GOTO       L_readUltrasonic19
	CLRF       R0+0
	CLRF       R0+1
	GOTO       L_end_readUltrasonic
L_readUltrasonic19:
;MyProject.c,177 :: 		}
	GOTO       L_readUltrasonic17
L_readUltrasonic18:
;MyProject.c,179 :: 		TMR1H = 0; TMR1L = 0;
	CLRF       TMR1H+0
	CLRF       TMR1L+0
;MyProject.c,180 :: 		T1CON = 0x01;
	MOVLW      1
	MOVWF      T1CON+0
;MyProject.c,182 :: 		timeout = 0xFFFF;
	MOVLW      255
	MOVWF      readUltrasonic_timeout_L0+0
	MOVLW      255
	MOVWF      readUltrasonic_timeout_L0+1
;MyProject.c,183 :: 		while (PORTB & ECHO) {
L_readUltrasonic20:
	BTFSS      PORTB+0, 1
	GOTO       L_readUltrasonic21
;MyProject.c,184 :: 		if (--timeout == 0) {
	MOVLW      1
	SUBWF      readUltrasonic_timeout_L0+0, 1
	BTFSS      STATUS+0, 0
	DECF       readUltrasonic_timeout_L0+1, 1
	MOVLW      0
	XORWF      readUltrasonic_timeout_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__readUltrasonic56
	MOVLW      0
	XORWF      readUltrasonic_timeout_L0+0, 0
L__readUltrasonic56:
	BTFSS      STATUS+0, 2
	GOTO       L_readUltrasonic22
;MyProject.c,185 :: 		T1CON = 0x00;
	CLRF       T1CON+0
;MyProject.c,186 :: 		return 0;
	CLRF       R0+0
	CLRF       R0+1
	GOTO       L_end_readUltrasonic
;MyProject.c,187 :: 		}
L_readUltrasonic22:
;MyProject.c,188 :: 		}
	GOTO       L_readUltrasonic20
L_readUltrasonic21:
;MyProject.c,190 :: 		T1CON = 0x00;
	CLRF       T1CON+0
;MyProject.c,191 :: 		time = (TMR1H << 8) | TMR1L;
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       TMR1L+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;MyProject.c,193 :: 		return time / 58;
	MOVLW      58
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Div_16X16_U+0
;MyProject.c,194 :: 		}
L_end_readUltrasonic:
	RETURN
; end of _readUltrasonic

_adel_delay:

;MyProject.c,197 :: 		void adel_delay(unsigned int ms) {
;MyProject.c,199 :: 		for (i = 0; i < ms; i++) {
	CLRF       R1+0
	CLRF       R1+1
L_adel_delay23:
	MOVF       FARG_adel_delay_ms+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__adel_delay58
	MOVF       FARG_adel_delay_ms+0, 0
	SUBWF      R1+0, 0
L__adel_delay58:
	BTFSC      STATUS+0, 0
	GOTO       L_adel_delay24
;MyProject.c,200 :: 		for (j = 0; j < 111; j++) NOP();
	CLRF       R3+0
	CLRF       R3+1
L_adel_delay26:
	MOVLW      0
	SUBWF      R3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__adel_delay59
	MOVLW      111
	SUBWF      R3+0, 0
L__adel_delay59:
	BTFSC      STATUS+0, 0
	GOTO       L_adel_delay27
	NOP
	INCF       R3+0, 1
	BTFSC      STATUS+0, 2
	INCF       R3+1, 1
	GOTO       L_adel_delay26
L_adel_delay27:
;MyProject.c,199 :: 		for (i = 0; i < ms; i++) {
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
;MyProject.c,201 :: 		}
	GOTO       L_adel_delay23
L_adel_delay24:
;MyProject.c,202 :: 		}
L_end_adel_delay:
	RETURN
; end of _adel_delay

_adel2_delay:

;MyProject.c,203 :: 		void adel2_delay(unsigned int ms) {
;MyProject.c,205 :: 		for (i = 0; i < ms; i++) {
	CLRF       R1+0
	CLRF       R1+1
L_adel2_delay29:
	MOVF       FARG_adel2_delay_ms+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__adel2_delay61
	MOVF       FARG_adel2_delay_ms+0, 0
	SUBWF      R1+0, 0
L__adel2_delay61:
	BTFSC      STATUS+0, 0
	GOTO       L_adel2_delay30
;MyProject.c,206 :: 		for (j = 0; j < 111; j++) NOP();
	CLRF       R3+0
	CLRF       R3+1
L_adel2_delay32:
	MOVLW      0
	SUBWF      R3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__adel2_delay62
	MOVLW      111
	SUBWF      R3+0, 0
L__adel2_delay62:
	BTFSC      STATUS+0, 0
	GOTO       L_adel2_delay33
	NOP
	INCF       R3+0, 1
	BTFSC      STATUS+0, 2
	INCF       R3+1, 1
	GOTO       L_adel2_delay32
L_adel2_delay33:
;MyProject.c,205 :: 		for (i = 0; i < ms; i++) {
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
;MyProject.c,207 :: 		}
	GOTO       L_adel2_delay29
L_adel2_delay30:
;MyProject.c,208 :: 		}
L_end_adel2_delay:
	RETURN
; end of _adel2_delay

_adel_us_delay:

;MyProject.c,210 :: 		void adel_us_delay(unsigned int us) {
;MyProject.c,212 :: 		for (i = 0; i < us; i++) NOP(); NOP(); NOP(); NOP();
	CLRF       R1+0
	CLRF       R1+1
L_adel_us_delay35:
	MOVF       FARG_adel_us_delay_us+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__adel_us_delay64
	MOVF       FARG_adel_us_delay_us+0, 0
	SUBWF      R1+0, 0
L__adel_us_delay64:
	BTFSC      STATUS+0, 0
	GOTO       L_adel_us_delay36
	NOP
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
	GOTO       L_adel_us_delay35
L_adel_us_delay36:
	NOP
	NOP
	NOP
;MyProject.c,213 :: 		}
L_end_adel_us_delay:
	RETURN
; end of _adel_us_delay
