#line 1 "C:/Users/adeli/OneDrive/Desktop/New folder/MyProject.c"
#line 20 "C:/Users/adeli/OneDrive/Desktop/New folder/MyProject.c"
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


volatile unsigned char lineSensorsTriggered = 0;


interrupt() {
 while (!(PORTB &  0x04 ) && !(PORTB &  0x08 )) {

 lineSensorsTriggered = 1;
 moveBackward();
 }
 lineSensorsTriggered = 0;
 INTCON &= ~0x02;
}


void main() {
 unsigned int distance, potValue;

 adel_delay(5000);
 init();
 initPWM();
 INTCON |= 0x90;

 while (1) {

 if (lineSensorsTriggered) {
 continue;
 }


 distance = readUltrasonic();
 potValue = readPotentiometer();


 setMotorSpeed(potValue);

 if (distance > 0 && distance < 30) {

 PORTA |=  0x10 ;
 moveForward();
 } else if (PORTB &  0x04 ) {

 moveBackwardRight();
 } else if (PORTB &  0x08 ) {

 moveBackwardLeft();
 } else {

 PORTA &= ~ 0x10 ;
 searchAround();
 }

 adel_delay(100);
 }
}


void init() {
 TRISB = 0x0E;
 TRISD = 0x00;
 TRISA = 0xFF;
 TRISC = 0xF9;

 ADCON0 = 0x41;
 ADCON1 = 0x80;
 T1CON = 0x00;

}


void initPWM() {

 T2CON = 0x04;
 PR2 = 0xFF;
 TMR2 = 0x00;


 CCP1CON = 0x0C;
 CCP2CON = 0x0C;


 T2CON |= 0x04;
}


void setMotorSpeed(unsigned int speed) {
 unsigned char pwm_value = speed / 4;

 CCPR1L = pwm_value;
 CCPR2L = pwm_value;
}


unsigned int readPotentiometer() {
 ADCON0 &= 0xC7;
 ADCON0 |= ( 0  << 3);
 adel_us_delay(20);
 ADCON0 |= 0x04;
 while (ADCON0 & 0x04);
 return ((ADRESH << 8) | ADRESL);
}


void moveForward() {
 PORTD =  0x01  |  0x04  |  0x10  |  0x40 ;
}


void moveBackward() {
 PORTD =  0x02  |  0x08  |  0x20  |  0x80 ;
 adel2_delay(500);
}


void moveBackwardRight() {
 PORTD =  0x02  |  0x20  |  0x04  |  0x40 ;
 adel_delay(500);
}


void moveBackwardLeft() {
 PORTD =  0x08  |  0x80  |  0x01  |  0x10 ;
 adel_delay(500);
}


void searchAround() {
 PORTD =  0x01  |  0x10  |  0x08  |  0x80 ;
 adel_delay(500);
}


unsigned int readUltrasonic() {
 unsigned int time = 0;
 unsigned int timeout;

 PORTB |=  0x01 ;
 adel_us_delay(10);
 PORTB &= ~ 0x01 ;

 timeout = 0xFFFF;
 while (!(PORTB &  0x02 )) {
 if (--timeout == 0) return 0;
 }

 TMR1H = 0; TMR1L = 0;
 T1CON = 0x01;

 timeout = 0xFFFF;
 while (PORTB &  0x02 ) {
 if (--timeout == 0) {
 T1CON = 0x00;
 return 0;
 }
 }

 T1CON = 0x00;
 time = (TMR1H << 8) | TMR1L;

 return time / 58;
}


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
