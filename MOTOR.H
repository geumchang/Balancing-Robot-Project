#ifndef MOTOR_H_
#define MOTOR_H_



#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>

#define ENABLE_A  PB4   // OC0 핀 for A모터 (PWM)
#define IN1_A     PA0   // IN1_A 핀 for A모터(direction)
#define IN2_A     PA1   // IN2_A 핀 for A모터(direction)
#define ENABLE_B  PB5   // OC1A 핀 for B모터 (PWM)
#define IN3_B     PA6   // IN3_B 핀 for B모터 (direction)
#define IN4_B     PA7   // IN4_B 핀 for B모터 (direction)

#define speedDC 255

void setup()
{
   
   DDRA |=  (1 << IN1_A) | (1 << IN2_A);
   DDRA |=  (1 << IN3_B) | (1 << IN4_B);
   // FAST PWM 모드(A모터)
  // TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS00);
   // FAST PWM 모드(B모터)
 
   
    
   
   TCCR1A = (1 << WGM11) | (1 << COM1A1);
   TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); //16비트 fast pwm모드 분주비 8
   
    ICR1 = 65535; // PWM 주기 설정

   DDRB |= (1 << PB4);  // OC0 핀을 출력으로(A모터)
   DDRB |= (1 << PB5);  // OC1A 핀을 출력으로(B모터)
   //분주비 둘다 1
   
}

void motor_speed(int spd)
{
   // PWM duty cycle 0~255
   OCR0 = spd;  // A모터
   OCR1A = spd; // B모터        
   
 
}

void motor_dir(int dir)
{
   
   if (dir == 0) {//모터 A,B 모두 순방향
      PORTA = (1<< IN1_A)| (0<< IN2_A) |(1<<IN3_B) |(0<<IN4_B);
   }
   
   else if (dir == 1) {//모터 A: 순방향, 모터 B: 역방향
      PORTA = (1<< IN1_A)| (0<< IN2_A) |(0<<IN3_B) |(1<<IN4_B);
   }
   
   else if (dir == 2) {//모터 A: 역방향, 모터 B: 순방향
      PORTA = (0<< IN1_A)| (1<< IN2_A) |(1<<IN3_B) |(0<<IN4_B);
   }
   else if (dir == 3) {//모터 A,B 모두 역방향
      PORTA = (0<< IN1_A)| (1<< IN2_A) |(0<<IN3_B) |(1<<IN4_B);
      
   }
   
}





#endif /* MOTOR_H_ */


#ifndef MOTOR_H_
#define MOTOR_H_

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>

#define ENABLE_A  PB6   // OC1B 핀 for A모터 (PWM)
#define IN1_A     PA0   // IN1_A 핀 for A모터(direction)
#define IN2_A     PA1   // IN2_A 핀 for A모터(direction)
#define ENABLE_B  PB5   // OC1A 핀 for B모터 (PWM)
#define IN3_B     PA6   // IN3_B 핀 for B모터 (direction)
#define IN4_B     PA7   // IN4_B 핀 for B모터 (direction)



void setup()
{
   DDRA |=  (1 << IN1_A) | (1 << IN2_A);
   DDRA |=  (1 << IN3_B) | (1 << IN4_B);

   TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
   TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); //16비트 fast pwm모드 분주비 8

   ICR1 = 65535; // PWM 주기 설정

   DDRB |= (1 << ENABLE_A);  // OC1B 핀 출력으로(A모터)
   DDRB |= (1 << ENABLE_B);  // OC1A 핀 출력으로(B모터)
}

void motor_speed(int spd)
{
   // PWM duty cycle 0~65535
   OCR1B = spd;  // A모터
   OCR1A = spd; // B모터
   
   
}

void motor_dir(int dir)
{
   if (dir == 0) {//모터 A,B 모두 순방향
      PORTA = (1<< IN1_A)| (0<< IN2_A) |(1<<IN3_B) |(0<<IN4_B);
   }
   else if (dir == 1) {//모터 A: 순방향, 모터 B: 역방향
      PORTA = (1<< IN1_A)| (0<< IN2_A) |(0<<IN3_B) |(1<<IN4_B);
   }
   else if (dir == 2) {//모터 A: 역방향, 모터 B: 순방향
      PORTA = (0<< IN1_A)| (1<< IN2_A) |(1<<IN3_B) |(0<<IN4_B);
   }
   else if (dir == 3) {//모터 A,B 모두 역방향
      PORTA = (0<< IN1_A)| (1<< IN2_A) |(0<<IN3_B) |(1<<IN4_B);
   }
}

#endif /* MOTOR_H_ */
