#ifndef MPU_H_
#define MPU_H_



#include <avr/interrupt.h>

#define HIGH_SPI_CS() (PORTB |= (1 << PB0))  // SPI_CS를 HIGH로 설정
#define LOW_SPI_CS() (PORTB &= ~(1 << PB0)) // SPI_CS를 LOW로 설정

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
float angle_roll = 0.0;
float angle_pitch = 0.0;
unsigned char timer_count = 0;
float angle;





void timer2_init(void)
{
   TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
   OCR2 = 13; //13 -> 약1ms
   TIMSK = (1<< OCIE2);
}
ISR(TIMER2_COMP_vect)
{
   timer_count++;
}


//////////////////////////////
//엔코더 부분
/*
void interrupt_init(void)
{
   EIMSK = 0x30;
   EICRB = 0x0f; //rising edge
}

ISR(INT4_vect)
{
   if (!(PINE &= (1<<PE5))) //PE5 =0이면
   {
      angle += 0.09; //2체배 : 360/4000 = 0.09 (펄스 당 각도)
      
   }
   else
   {
      angle -= 0.09; // 역방향
   }
   if (angle >= 360 || angle <= -360)
   {
      angle = 0;
   }
   
   if (angle < 0)
   {
      angle = 360 - angle;
   }
}

ISR(INT5_vect)
{
   if (PINE &= (1<<PE4)) //PE4가 1이면
   {
      angle += 0.09;
      
   }
   else
   {
      angle -= 0.09;
   }
   if (angle >= 360 || angle <= -360)
   {
      angle = 0;
   }
   
   if (angle < 0)
   {
      angle = 360 - angle;
   }
}
*/
//////////////////////////////////////////////////////////////////////

enum MPU_9250_REG_ADRESS{
   GYRO_CONFIG = 0X03,   // 자이로(각속도)센서 설정을 위한 레지스터
   ACCEL_CONFIG = 0X04,  // 가속도센서 설정을 위한 레지스터
   ACCEL_CONFIG_2 = 0X1D,// 가속도 센서 추가 설정을 위한 레지스터
   
   /*측정된 가속도 정보를 저장하는 레지스터 */
   A_XOUT_H = 0X3B,
   A_XOUT_L = 0X3C,      
   
   A_YOUT_H = 0X3D,
   A_YOUT_L = 0X3E,
   
   A_ZOUT_H = 0X3F,
   A_ZOUT_L = 0X40,
   ///////////////////////////////////////
   
   /*측정된 각속도 정보를 저장하는 레지스터*/
   G_XOUT_H = 0X43,
   G_XOUT_L = 0X44,
   
   G_YOUT_H = 0X45,
   G_YOUT_L = 0X46,
   
   G_ZOUT_H = 0X47,
   G_ZOUT_L = 0X48,
   ///////////////////////////////////////
   
   //센서의 디지털 경로 초기화 설정을 위한 레지스터
   SIGNAL_PATH_RESET = 0X68,
   USER_CTRL = 0X6A,
   
   //MPU9250의 전원관리을 위한 레지스터
   PWR_MGMT_1 = 0X6B,
   PWR_MGMT_2 = 0X6C

};
//////////////////////////////////////////////////////////////////////////




/*SPI 초기화 함수*/
void MPU9250_SPI_init(void)
{
   DDRB |= 0x07;
   //SS(PB0),SCK(PB1),MOSI(PB2)-> 출력 | MISO(PB3)-> 입력
   SPCR = (1<<SPE) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (2<<SPR0);
   //SPI 활성화, 마스터 모드, 클록 上 설정, 클록 64분주 설정
}

/*SPI를 통한 데이터 쓰기 함수*/
void MPU9250_SPI_WRITE(char regAddress, char data)
{
   LOW_SPI_CS(); // 슬레이브 장치 선택
   SPDR = regAddress;  // 레지스터 주소 전달
   while (!(SPSR & (1 << SPIF))); // 송신 완료 대기
   SPDR = data;    // 데이터 전달
   while (!(SPSR & (1 << SPIF)));  // 송신 완료 대기
   HIGH_SPI_CS(); // 슬레이브 장치 선택 해제
}


/*SPI를 통한 데이터 읽기 함수*/
unsigned char MPU9250_SPI_READ(char regAddress)
{
   unsigned char data;
   LOW_SPI_CS();  // 슬레이브 장치 선택
   SPDR = regAddress | 0x80;  // 레지스터 주소 전달(읽기)
   while (!(SPSR & (1 << SPIF))); // 송신 완료 대기
   SPDR = 0x00;  // 데이터 수신을 위한 더미 송신
   while (!(SPSR & (1 << SPIF))); // 송신 완료 대기
   data = SPDR;  // MISO로 입력된 데이터 획득
   HIGH_SPI_CS();  // 슬레이브 장치 선택 해제
   return data;  // 수신된 데이터 반환
}


void Get_Accel(int *prAx, int *prAy, int *prAz) //prAx,prAy,prAz : 3축 가속도 값을 저장할 변수의 참조 값
{
   unsigned char Accel_data[6];
   
   Accel_data[0] = MPU9250_SPI_READ(A_XOUT_H);
   Accel_data[1] = MPU9250_SPI_READ(A_XOUT_L);
   
   Accel_data[2] = MPU9250_SPI_READ(A_YOUT_H);
   Accel_data[3] = MPU9250_SPI_READ(A_YOUT_L);
   
   Accel_data[4] = MPU9250_SPI_READ(A_ZOUT_H);
   Accel_data[5] = MPU9250_SPI_READ(A_ZOUT_L);
   
   *prAx = ((Accel_data[0] <<8) | Accel_data[1]);      //가속도 각속도 출력 = 2bytes 이므로
   *prAy = ((Accel_data[2] <<8) | Accel_data[3]);
   *prAz = ((Accel_data[4] <<8) | Accel_data[5]);
}




void Get_Gyro(int *prGx, int *prGy, int *prGz) 
//prGx,prGy,prGz : 3축 각속도 값을 저장할 변수의 참조 값
{
   unsigned char Gyro_data[6];
   
   Gyro_data[0] = MPU9250_SPI_READ(G_XOUT_H);
   Gyro_data[1] = MPU9250_SPI_READ(G_XOUT_L);
   Gyro_data[2] = MPU9250_SPI_READ(G_YOUT_H);
   Gyro_data[3] = MPU9250_SPI_READ(G_YOUT_L);
   Gyro_data[4] = MPU9250_SPI_READ(G_ZOUT_H);
   Gyro_data[5] = MPU9250_SPI_READ(G_ZOUT_L);

   *prGx = ((Gyro_data[0] <<8) | Gyro_data[1]);
   *prGy = ((Gyro_data[2] <<8) | Gyro_data[3]);   //자이로 원데이터
   *prGz = ((Gyro_data[4] <<8) | Gyro_data[5]);



   
   ///////////////////////////////////////
}

//센서 초기화 함수

void MPU9250_Init(void)
{
   MPU9250_SPI_WRITE(PWR_MGMT_1, 0X80);
   MPU9250_SPI_WRITE(SIGNAL_PATH_RESET, 0X06);
   MPU9250_SPI_WRITE(USER_CTRL, 0X01);   
   MPU9250_SPI_WRITE(GYRO_CONFIG, 0X00);
   MPU9250_SPI_WRITE(ACCEL_CONFIG, 0X00);//가속도 Scale 초기화
   MPU9250_SPI_WRITE(PWR_MGMT_2, 0X00);   
}

//변환함수

void Conv_Value_Acc(float *pAx, float *pAy,float *pAz, int rAx, int rAy, int rAz)
{
   *pAx = (float)rAx/16384; //가속도 g단위 변환
   *pAy = (float)rAy/16384; //가속도 g단위 변환
   *pAz = (float)rAz/16384; //가속도 g단위 변환
}

void Conv_Value_Gyro(float *pGx, float *pGy,float *pGz, int rGx, int rGy, int rGz)
{
   *pGx = (float)rGx/131.072; //각속도 (degree per second)단위 변환
   *pGy = (float)rGy/131.072; //각속도 (degree per second)단위 변환
   *pGz = (float)rGz/131.072; //각속도 (degree per second)단위 변환
}



void Accel_degreeRPY(float rax, float ray, float raz, float rgx, float rgy, float rgz)
{
   ////// 가속도계 데이터를 사용하여 roll, pitch 계산 //////////
   roll = atan2(ray, sqrt(rax * rax + raz * raz)) * 180/M_PI; //라디안으로 얻기위해
   pitch= atan2(rax, sqrt(ray * ray + raz * raz))* 180/M_PI;
}

//*********************Gyro*********************//

void Gyro_Yaw(double *angle_yaw, int Gz)
{
   *angle_yaw = *angle_yaw - ((double)Gz *0.008);
}

void Gyro_pitch(double *angle_pitch, int Gy)
{
   *angle_pitch = *angle_pitch - ((double)Gy * 0.005);
}

void Gyro_roll(double *angle_roll, int Gx)
{
   *angle_roll = *angle_roll - (double)Gx * 0.005;
}




//*********************Filter*********************//
void complement_Filter_roll(double *angle, double angle_pre, int Gx, double accel_deg)
{
   //complementary Filter 공식
   //angle = alpha*[(Gyro_data) + Delta_t + previous_Angle)] + (1-alpha)*Accel_angle
   //alpha = T/T+delta_t     //T: 가속도 센서 노이즈 시간의 스케일보다 월등히 큰 수
      *angle = (0.96*(*angle +Gx*0.005)) + (0.04* accel_deg);
   //*angle = (0.9996*(angle_pre) + 0.9996*(double)Gx *0.05) + (0.0004 * accel_deg);
}
void complement_Filter_pitch(double *angle, double angle_pre, int Gy, double accel_deg)
{
   //complementary Filter 공식
   //angle = alpha*[(Gyro_data) + Delta_t + previous_Angle)] + (1-alpha)*Accel_angle
   //alpha = T/T+delta_t     //T: 가속도 센서 노이즈 시간의 스케일보다 월등히 큰 수
   *angle = (0.9996*(angle_pre) + 0.9996*(double)Gy *0.05) + (0.0004 * accel_deg);
}



#endif /* MPU_H_ */
