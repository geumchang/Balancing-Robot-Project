#ifndef USART1_H_
#define USART1_H_






/*USART를 이용한 출력 위한 USART 설정*/
void Init_USART1(void)
{
   UCSR1B = (1 << TXEN1);  // 송신기 활성화
   UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
   // 8비트 데이터, 패리티 없음, 1 스톱 비트
   UBRR1H = 0;
   UBRR1L = 7;
}

void putch_USART1(char data)
{
   while(!(UCSR1A& (1 << UDRE1)));
   UDR1 = data;
}

void puts_USART1(char *str)
{
   while(*str != 0)
   {
      putch_USART1(*str);
      str++;
   }
}



#endif /* USART1_H_ */
