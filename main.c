/*
 * balanerobot_16.c
 *
 * Created: 2024-01-25 오전 9:24:49
 * Author : PRO
 */ 



#define F_CPU 14745600UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include "motor.h"
#include "usart1.h"
#include "mpu.h"   
// 3368 1568
#define KP 1118.0 // Proportional gain
#define KI 0.0 // Integral gain
#define KD  80.0 // Derivative gain


double error_roll_prev = 0.0, error_pitch_prev = 0.0;  // 이전 오차값
//double error_roll_sum = 0.0, error_pitch_sum = 0.0;  // 오차의 누적값

double P_roll =0.0;
double I_roll = 0.0;
double D_roll =0.0;

double output_roll = 0.0;

int rax =0, ray=0, raz =0;
int rgx =0, rgy=0, rgz =0;

//int rgx_prev = 0, rgy_prev = 0, rgz_prev = 0;

//기준단위 변환을 위한 데이터 저장을 위한 변수선언
float ax=0.0, ay=0.0, az=0.0;
float gx =0.0, gy=0.0, gz=0.0;

char accel_data[100];
char gyro_data[100];
char euler_data[100] = {0,};
//char euler_data[100];

// Gyro yaw, roll, pitch 변수
double Gyro_yaw_angle=0.0;
double Gyro_roll_angle=0.0;
double Gyro_pitch_angle=0.0;

// 전단계 yaw, roll, pitch 각도 변수
double angle_yaw_previ=0.0 ;
double angle_roll_previ=0.0;
double angle_pitch_previ=0.0;




double complementary_roll = 0.0;
double complementary_pitch = 0.0;

double sensor_roll = 0.0, sensor_pitch = 0.0;
double target_roll = 0.0, target_pitch = 0.0;
double error_roll = 0.0;


void PID_control(double sensor_roll) {
   // roll에 대한 PID 제어
   
    error_roll = target_roll - sensor_roll;  // 현재 오차 계산
   
   //////////// DEAD LINE 지정///////////////
   //if (error_roll > 45) error_roll = 45;
   //if (error_roll < -45) error_roll = -45;
   //////////////////////////////////////////
   
   P_roll = KP * error_roll;  // 비례항 계산
   //error_roll_sum += error_roll;  // 오차 누적
   I_roll += KI * error_roll;  // 적분항 계산
   D_roll = KD * (error_roll - error_roll_prev);  // 미분항 계산
   
   output_roll = P_roll + I_roll + D_roll;  // 출력값 계산
   motor_control(output_roll);
   error_roll_prev = error_roll;  // 이전 오차값 갱신
}

void motor_control(double output_roll) {
   // output_roll을 사용하여 모터의 속도를 제어
   // output을 -65535 ~ 65535의 범위로 조정
   int motor_speed1 = (int)(output_roll);
   
   //////////// 속도 DEAD LINE 지정/////////////
   if(0 < motor_speed1 && motor_speed1 < 3500) motor_speed1 = 3500;
   if(-3500 < motor_speed1 && motor_speed1 < 0) motor_speed1 = -3500;
   if (motor_speed1 > 65535) motor_speed1 = 65535;
   if (motor_speed1 < -65535) motor_speed1 = -65535;
   ////////////////////////////////////////////
   
   
   if (motor_speed1 >= 0) motor_dir(3);
   else motor_dir(0);
  
   // 모터의 속도를 설정합니다.
   motor_speed(abs(motor_speed1));
   
 
}
 


 
 
 
void loop()
{
  
   
   if (timer_count >= 5) //sampling
   {

      Accel_degreeRPY(rax, ray, raz, rgx, rgy, rgz);
     Get_Accel(&rax, &ray, &raz);
      Get_Gyro(&rgx, &rgy, &rgz);
      
      //단위 변환
      Conv_Value_Acc(&ax, &ay, &az, rax, ray, raz);
      Conv_Value_Gyro(&gx,&gy, &gz, rgx, rgy, rgz);
      
      Gyro_Yaw(&Gyro_yaw_angle,gz);
      Gyro_pitch(&Gyro_pitch_angle,gy);
      Gyro_roll(&Gyro_roll_angle,gx);
      
      complement_Filter_roll(&complementary_roll,angle_roll_previ,gx,roll);
      //complement_Filter_pitch(&complementary_pitch,angle_pitch_previ,gy,pitch);
      
     
      
      sensor_roll =  complementary_roll;
      
      PID_control(sensor_roll);
   
   
     sprintf(euler_data, "%.2f\t %.2f\t %.2f\t %.2f\n", sensor_roll, P_roll, D_roll, output_roll);
      
      puts_USART1(euler_data);
                
     timer_count = 0;
      
   }
   
}


 //모터가 움직이지 않는 구간 DEADZONE의 범위를 알기위한 함수
void deadzonefind()
{
   
   for(int j =2000; j<65536; j++)
   {
      motor_dir(0);
      motor_speed(j);
      sprintf(euler_data, " %d\n", j);
       
      puts_USART1(euler_data);
      _delay_ms(40);
      
      
   }
}

void main(void)
{  
   timer_count = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

   setup();

  
   timer2_init();
   Init_USART1();
   MPU9250_SPI_init();
   MPU9250_Init();
   sei();
   

   
   
   
   while(1)
   {
   //deadzonefind();
 

   
    
   loop();
  
      

   }
   }
