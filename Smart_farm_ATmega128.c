/////============================================TMP CDS SM 설정=======================================////

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <util/delay.h> 
#include "lcd.h"


#define  Avg_Num     4         //  이동 평균 갯수 
#define  Amp_Gain   11         //  증폭기 이득  

#define  TMP_Mode   0          //  온도센서 디스플레이 모드   
#define  CDS_Mode   1          //  CDS 센서 디스플레이 모드 
#define  SM_Mode    2		   //  토양수분센서 디스플레이 모드
#define  Running_Mode 3		   //  초기 실행모드

//===================================================================================================

void init_serial(void) ;  //  Serial 토신포트 초기화

void SerialPutChar(char ch);
void SerialPutString(char str[]);


void HexToDec( unsigned short num, unsigned short radix); 

char NumToAsc( unsigned char Num ); 

static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 

void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // 부호없는 정수형 변수를 10진수 형태로 LCD 에 디스플레이 
void Display_TMP_LCD( unsigned int tp  )  ;                           // 온도를 10진수 형태로 LCD 에 디스플레이 

void msec_delay(unsigned int n);
void usec_delay(unsigned int n);

static volatile unsigned short TMP_sensor_ouput= 0,  TMP_sensor_ouput_avg = 0, TMP_sensor_ouput_avg_C = 0 ; 
static volatile unsigned short CDS_sensor_ouput= 1000,  CDS_sensor_ouput_avg = 1000 ; 
static volatile unsigned short SM_sensor_ouput=1000,SM_sensor_ouput_avg = 1000;

static volatile unsigned char int_num = 0,  Sensor_Flag = Running_Mode ;

static volatile  char  recv_cnt = 0, rdata=0, new_recv_flag = 0  ;  
  
//======================================DC모터 셋팅====================================================
 void DC_Motor_Run_Fwd_L( short duty );    // 블라인드 DC 모터 정회전(PWM구동) 함수 
void DC_Motor_Run_Rev_L( short duty );    // 블라인드 DC 모터 역회전(PWM구동) 함수  
void DC_Motor_Stop_L( void );             // 블라인드 DC 모터 정지 함수  
 
void DC_Motor_Run_Fwd_R( short duty );    // 워터펌프 정회전(PWM구동) 함수 
void DC_Motor_Run_Rev_R( short duty );    // 워터펌프 역회전(PWM구동) 함수  
void DC_Motor_Stop_R( void );             // 워터펌프 정지 함수  

void DC_Motor_PWM_L( short Vref );        // 블라인드 DC 모터 PWM 신호 발생 함수  
                                          // 정토크(Vref>0), 역토크(Vref<0), 영토크(Vref=0) 모두 포함 
void DC_Motor_PWM_R( short Vref );        // 워터펌프 PWM 신호 발생 함수  
                                          // 정토크(Vref>0), 역토크(Vref<0), 영토크(Vref=0) 모두 포함 

static volatile short  Vmax = 0, Vmax_L = 0, Vmax_R = 0  ; 

//=======================================================================================================

static volatile unsigned short Mode = 0, Mode2 = 1, SA_mode = 0, Limit = 0, LED_Mode = 0;


int main() 
{   

	short duty_L = 0, duty_R = 0; 

	DDRB |= 0x10;     // LED (PB4 : 출력설정 )
	PORTB |= 0x10;    // PB4  : High ( LED OFF)  


    init_serial() ;   // Serial Port (USART1) 초기화


	LcdInit();               // LCD 초기화
    UCSR1B |=  0x80  ;      // UART1 수신(RX) 완료 인터럽트 허용
    LcdCommand(ALLCLR) ;

 
/*****   AD Converter **********/

	ADMUX &= ~0xE0;    //  ADC 기준전압 = AREF ,   ADC 결과 오른쪽정렬 
	ADCSRA |= 0x87;     // ADC enable, Prescaler = 128



/**** Timer0 Overflow Interrupt  ******/
/**************************************/
	TCCR0 = 0x00; 
    TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64use

	TIMSK = 0x01;  // Timer0 overflow interrupt enable 
	sei();         // Global Interrupt Enable 


	TCCR0 |= 0x07; // Clock Prescaler N=1024 (Timer 0 Start)



/*******************DC 모터 구동*********************************/





	DDRB |= 0x20;   // 블라인드 모터구동신호 + 단자:  PWM 포트( pin: OC1A(PB5) )   --> 출력 설정 
	DDRA |= 0x01;   // 블라인드 모터구동신호 - 단자 : 범용 입/출력포트(pin : PA0 ) --> 출력 설정 

	DDRB |= 0x40;   // 워터펌프 구동신호 + 단자:  PWM 포트( pin: OC1A(PB6) )   --> 출력 설정 
	DDRA |= 0x02;   // 워터펌프 구동신호 - 단자 : 범용 입/출력포트(pin : PA1 ) --> 출력 설정 


// 블라인드, 워터펌프 구동 PWM 신호 ( pin: OC1A(PB5)), OC1B(PB6) ),   Timer1,  PWM signal (period= 200 usec )

	TCCR1A = 0xA2;    // OC1A(PB5)), OC1B(PB6) :  PWM 포트 설정,   Fast PWM ( mode 14 )
	TCCR1B = 0x1B;    // 64 분주 타이머 1 시작 (내부클럭 주기 =  64/(16*10^6) = 4 usec ),  Fast PWM ( mode 14 ) 
	ICR1 = 50;        // PWM 주기 = 50 * 4 usec = 200 usec

    Vmax_L = ICR1; 
    Vmax_R = ICR1; 

    Vmax =  ICR1 ;

//////////////////////////////////////////////////////////////////
    duty_L = 0;
	OCR1A = duty_L;      //  블라인드 모터 정지: : OC1A(PB5) PWM duty = 0 설정 
	
    duty_R = 0;
	OCR1B = duty_R;      //  워터펌프 정지: : OC1B(PB6) PWM duty = 0 설정 	
	 
//////////////////////////////////////////////////////////////////

    duty_L = 50;         // 블라인드 모터 속도 설정,   최대 = Vmax = 50,  최소 = 0 

    duty_R = 50;         // 워터펌프 속도 설정, 최대 = Vmax = 50,  최소 = 0 
///////////////////////////////////////////////////////////////////////////
/******************************************************************/



		 
	while (1) 
	{ 

		if( new_recv_flag == 1 )      // 1 문자 수신완료 시 
		{ 

		//////////////  명령어 처리   //////////////


////////////////////////////////////////////////센서값 출력부/////////////////////////////////////////////////////////////////////

			if( rdata == '0' )          // 문자 0 이 수신되면 
			{
		           HexToDec(TMP_sensor_ouput_avg_C,10);   // 온도센서값 TMP_sensor_ouput_avg_C 십진수로 변환

                   SerialPutString( "Temperature = ") ; //  메시지 전송

                   SerialPutChar( NumToAsc(cnumber[2]));   // 10자리 (변수 TMP_sensor_ouput_avg_C) 전송
                   SerialPutChar( NumToAsc(cnumber[1]));   // 1 자리 
                   SerialPutChar( '.' );
                   SerialPutChar( NumToAsc(cnumber[0]));   // 0.1 자리
                   SerialPutChar( 'C' );
                   SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

				   Sensor_Flag = TMP_Mode;


			}
			else if( rdata == '1' )     // 문자 1 이 수신되면
			{
		           HexToDec(CDS_sensor_ouput_avg,10);   // 광량센서 값 CDS_sensor_ouput_avg 십진수로 변환

                   SerialPutString( "CDS Sensor Value = ") ; //  메시지 전송

                   SerialPutChar( NumToAsc(cnumber[3]));  //  1000자리, 변수 recv_cnt 값 전송
                   SerialPutChar( NumToAsc(cnumber[2]));  //  100자리
                   SerialPutChar( NumToAsc(cnumber[1]));  //  10자리
                   SerialPutChar( NumToAsc(cnumber[0]));  //  1자리
                   SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

				   Sensor_Flag = CDS_Mode;

			}

			else if( rdata == '2')      // 문자 2 가 수신되면
			{
		           HexToDec(SM_sensor_ouput_avg,10);   // 토양수분센서 값 SM_sensor_ouput_avg 십진수로 변환

                   SerialPutString( "SM Sensor Value = ") ; //  메시지 전송

                   SerialPutChar( NumToAsc(cnumber[3]));  //  1000자리, 변수 recv_cnt 값 전송
                   SerialPutChar( NumToAsc(cnumber[2]));  //  100자리
                   SerialPutChar( NumToAsc(cnumber[1]));  //  10자리
                   SerialPutChar( NumToAsc(cnumber[0]));  //  1자리
                   SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함
						   
				   Sensor_Flag = SM_Mode;

////////////////////////////////////////////////블라인드 제어부////////////////////////////////////////////////////
			} 
			else if( rdata == '3')      // 문자 3 가 수신되면
			{
				   if(Limit == 0)
				   {
				   		Limit = 1;
		           		DC_Motor_PWM_L( -duty_L );      // 블라인드 DC Motor 역회전 열기
		           		msec_delay( 5000 ); 
				   		DC_Motor_Stop_L();

                   		SerialPutString( "OPEN") ; //  메시지 전송
                   		SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함
				   }
				   else if(Limit == 1)
				   {
				   		SerialPutString( "Blind was allready opened!!  Try again.\n" ); //  명령 오류 메시지 전송
				   }	
			} 
			else if( rdata == '4')      // 문자 4 가 수신되면
			{
		          if(Limit == 1)
				  {
				  		Limit = 0;
				  		DC_Motor_PWM_L( duty_L );     // 블라인드 DC Motor 정회전 닫기 
				  		msec_delay( 4200 ); 
				  		DC_Motor_Stop_L();

                  		SerialPutString( "CLOSE") ; //  메시지 전송
                  		SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함
				  }
				  else if(Limit == 0)
				  {
						SerialPutString( "Blind was allready closed!!!  Try again.\n" ); //  명령 오류 메시지 전송
				  }
			}

////////////////////////////////////////////////LED 수동 제어부////////////////////////////////////////////////////////

			else if( rdata == '5')      // 문자 5 가 수신되면
			{
				  LED_Mode++;
				  if(LED_Mode==2) LED_Mode=0;
				  if(LED_Mode==1)
				  {	
			  		PORTB |= 0x10;   // PB4  : High ( LED ON )
				
                  	SerialPutString( "LED ON,LED Auto OFF") ; //  메시지 전송
                  	SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함
				  	Mode = 1;
				  }
				  else if (LED_Mode==0)
				  {
				  	PORTB &= ~0x10;   // PB4  : Low ( LED OFF )
				
                  	SerialPutString( "LED OFF,LED Auto OFF") ; //  메시지 전송
                  	SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함
				  	Mode = 1;
				  }
			}
			else if( rdata == '6')      // 문자 6 가 수신되면
			{
			
				  PORTB &= ~0x10;   // PB4  : Low ( LED OFF )
				  msec_delay(100);
				  Mode = 0;
                  SerialPutString( "LED OFF,LED Auto ON") ; //  메시지 전송
                  SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

			} 

//////////////////////////////////워터펌프 수동 제어부/////////////////////////////////////////////////////////////

			else if( rdata == '7')      // 문자 7 가 수신되면
			{
				  DC_Motor_PWM_R( duty_R );      // 워터펌프 정회전  
		      
				    
			 			   
                  SerialPutString( "Water ON,") ; //  메시지 전송
                  SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송

			} 

			else if( rdata == '8')      // 문자 8 가 수신되면
			{
			
				  DC_Motor_PWM_R( 0 );         // 워터펌프 정지
						   
                  SerialPutString( "Water OFF") ; //  메시지 전송
                  SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

			} 

			else if( rdata == '9')      // 문자 9 가 수신되면
			{
				  if(Mode2 ==0)   
				  { 
					    DC_Motor_PWM_R( 0 );         // 워터펌프 정지			
					    Mode2 = 1;
					    SerialPutString( "Water Auto OFF ") ; //  메시지 전송
                        SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송
				  }
				  else if(Mode2 == 1)
				  {	
				    	Mode2 = 0;
						SerialPutString( "Water Auto ON ") ; //  메시지 전송
                        SerialPutChar('\n');            // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송
				  }

			}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			else if( rdata != 0xFF)    //  명령 오류 이면
			{

                  SerialPutString( "Command Error!!  Try again.\n" ); //  명령 오류 메시지 전송

			}


		    rdata = 0xFF;
            new_recv_flag = 0;      // 새 문자(명령) 수신 플래그 Reset
  
        }

///////////////////////LCD 온,광도 디스플레이//////////////////////

        if( Sensor_Flag == TMP_Mode )
 	    { 

                LcdCommand(ALLCLR) ;   //화면 지움 

	            LcdMove(0,0);  
	            LcdPuts("Temp Sensor:");

	            LcdMove(1,0);  
	       	    LcdPuts("TMP = ");

	            LcdMove(1,6);  
                Display_TMP_LCD( TMP_sensor_ouput_avg_C  ); 
     	        LcdPuts("C");
  
	    }
        else if( Sensor_Flag == CDS_Mode )
 	    { 
            	LcdCommand(ALLCLR) ;
	       		LcdMove(0,0);  
	       		LcdPuts("CDS Sensor:");

	       		LcdMove(1,0);  
	       		LcdPuts("CDS = ");

	       		LcdMove(1,6);  
           		Display_Number_LCD( CDS_sensor_ouput_avg, 4 ); 
        }
        else if( Sensor_Flag == SM_Mode )
 	   	{ 
          		LcdCommand(ALLCLR) ;
	       		LcdMove(0,0);  
	       		LcdPuts("SM Sensor:");

	       		LcdMove(1,0);  
	       		LcdPuts("SM = ");

	       		LcdMove(1,6);  
           		Display_Number_LCD( SM_sensor_ouput_avg, 4 ); 
        }
		else if( Sensor_Flag == Running_Mode )
		{ 
          		LcdCommand(ALLCLR) ;
	       		LcdMove(0,0);  
		   		LcdPuts("***Smart Farm***");
		   		LcdMove(1,0); 
		   		LcdPuts("****Running*****");
		}

/////////////////////CDS값에 따른 LED점등//////////////////////

	    if(Mode==0)
	    {
	
		    	if( CDS_sensor_ouput_avg >= 500 )
	   			{	
	       			PORTB &= ~0x10;   // PB4  : Low ( LED ON )  
            	}
	   			else if( CDS_sensor_ouput_avg < 500 )
	  			{	
	       			PORTB |= 0x10;   // PB4  : High ( LED OFF )  
           		}
	    }

 ///////////////////////SM값에 따른 펌프작동/////////////////////////

	
		if(Mode2 == 0)
		{
	  			if(SM_sensor_ouput_avg > 700 )
				{
					DC_Motor_PWM_R( duty_R );      // 오른쪽 DC Motor 정회전  
	    			msec_delay( 1000 );          // 1초간 회전
				}
				if(SM_sensor_ouput_avg <=700  )
				{
					DC_Motor_PWM_R( 0 );         // 오른쪽 DC Motor 정지
				}
		}
        msec_delay(200); // 시간지연 
	}
} 


/////////////////////////////////////////


// UART1 수신 인터럽트 서비스 프로그램

ISR(  USART1_RX_vect )
{
    rdata = UDR1;                    // 수신된 데이터를 변수 rdata에 저장
 
    SerialPutChar( rdata);           // Echo  수신된 데이터를 바로 송신하여 수신된 데이터가 정확한지 확인 
    SerialPutChar('\n');             // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

    recv_cnt++ ;                     // 수신된 데이터 바이트수 저장

    new_recv_flag = 1;               // 새 문자(명령) 수신 플래그 Set
}



// UART1 통신 초기화 프로그램 

void init_serial(void)
{
    UCSR1A = 0x00;                    //초기화
    UCSR1B = 0x18  ;                  //송수신허용,  송수신 인터럽트 금지
    UCSR1C = 0x06;                    //데이터 전송비트 수 8비트로 설정.
    
    UBRR1H = 0x00;
    UBRR1L = 103;                     //Baud Rate 9600 
}




//======================================
// 한 문자를 송신한다.
//======================================

void SerialPutChar(char ch)
{
	while(!(UCSR1A & (1<<UDRE)));			// 버퍼가 빌 때를 기다림
  	UDR1 = ch;								// 버퍼에 문자를 쓴다
}


//=============================================
// 문자열을 송신한다.
// 입력   : str - 송신한 문자열을 저장할 버퍼의 주소
//=============================================

void SerialPutString(char *str)
{

    while(*str != '\0')         // 수신된 문자가 Null 문자( 0x00 )가 아니면 
    {
        SerialPutChar(*str++);
    }
}







ISR(TIMER0_OVF_vect)   // Timer0 overflow interrupt( 10 msec)  service routine
{

    static unsigned short  time_index = 0,  count1 = 0, TMP_Sum = 0, CDS_Sum = 0,SM_Sum=0; 
    static unsigned short  TMP_sensor_ouput_buf[Avg_Num ], CDS_sensor_ouput_buf[Avg_Num ],SM_sensor_ouput_buf[Avg_Num ]   ; 


    unsigned char i = 0 ;


    TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64usec

    time_index++ ; 


    if( time_index == 25 )    // 샘플링주기 =  250 msec = 10msec x 25 
    {

       time_index = 0; 


      /**************   CDS Sensor signal detection(AD 변환) ************/

	   ADMUX &= ~0x1F;    //  ADC Chanel 0 : ADC0 선택

	   ADCSRA |= 0x40;   // ADC start 

	   while( ( ADCSRA & 0x10 ) == 0x00  ) ;  // Check if ADC Conversion is completed 
           ADCSRA |= 0x10  ;   // 변환완료 비트 리셋 

	   CDS_sensor_ouput = ADC;   // AD 변환결과를 광량값 변수에 저장
 
     /******************************************************/ 


      /**************   Temperature Sensor signal detection(AD 변환) ************/

	   ADMUX &= ~0x1F;    //  ADC Chanel 선택 Clear
	   ADMUX |= 0x01;     //  ADC Chanel 1 : ADC 1 선택

	   ADCSRA |= 0x40;    // ADC start 

	   while( ( ADCSRA & 0x10 ) == 0x00  ) ;  // Check if ADC Conversion is completed 
           ADCSRA |= 0x10  ;   // 변환완료 비트 리셋 

	   TMP_sensor_ouput = ADC;      // ADC Conversion 이 완료되었으면 ADC 결과 저장 
 
     /******************************************************/ 

      /**************   Soil Moisture Sensor signal detection(AD 변환) ************/

	   ADMUX &= ~0x1F;    //  ADC Chanel 선택 Clear
	   ADMUX |= 0x02;     //  ADC Chanel 1 : ADC 1 선택

	   ADCSRA |= 0x40;    // ADC start 

	   while( ( ADCSRA & 0x10 ) == 0x00  ) ;  // Check if ADC Conversion is completed 
           ADCSRA |= 0x10  ;   // 변환완료 비트 리셋 

	   SM_sensor_ouput = ADC;      // ADC Conversion 이 완료되었으면 ADC 결과 저장 
 
     /******************************************************/ 



   ////////////////////////////////////////////////////////////////////
   //////////                                               /////////// 
   //////////  Avg_Num(4개) 개씩 이동 평균(Moving Average)  ///////////
   //////////                                               ///////////
   ////////////////////////////////////////////////////////////////////

	   if( count1 <= ( Avg_Num -1 ) )
	   {

          CDS_sensor_ouput_buf[ count1 ] = CDS_sensor_ouput ;
	      CDS_Sum +=  CDS_sensor_ouput_buf[ count1 ] ; 

          ////////////////////////////////////////////////////

          TMP_sensor_ouput_buf[ count1 ] = TMP_sensor_ouput ;
	      TMP_Sum +=  TMP_sensor_ouput_buf[ count1 ] ; 
		 //////////////////////////////////////////////////////
		  SM_sensor_ouput_buf[ count1 ] = SM_sensor_ouput ;
	      SM_Sum +=  SM_sensor_ouput_buf[ count1 ] ;	


	      count1++ ; 

	   } 
	   else
	   {

          CDS_Sum +=  CDS_sensor_ouput  ;	       // 가장 최근 값 더하고  
          CDS_Sum -=  CDS_sensor_ouput_buf[ 0 ] ;   // 가장 오랜된 값 빼고 

          CDS_sensor_ouput_avg = CDS_Sum / Avg_Num ;     // 4개 이동 평균 

          for( i = 0; i <= (Avg_Num - 2) ; i++ )
	      {
              CDS_sensor_ouput_buf[ i ]  = CDS_sensor_ouput_buf[ i+1 ] ;
	      } 

          CDS_sensor_ouput_buf[ Avg_Num - 1 ]  = CDS_sensor_ouput ;  
	      ////////////////////////////////////////////////////////////////

		  SM_Sum +=  SM_sensor_ouput  ;	       // 가장 최근 값 더하고  
          SM_Sum -=  SM_sensor_ouput_buf[ 0 ] ;   // 가장 오랜된 값 빼고 

          SM_sensor_ouput_avg = SM_Sum / Avg_Num ;     // 4개 이동 평균 

          for( i = 0; i <= (Avg_Num - 2) ; i++ )
		  {
              SM_sensor_ouput_buf[ i ]  = SM_sensor_ouput_buf[ i+1 ] ;
		  } 

          SM_sensor_ouput_buf[ Avg_Num - 1 ]  = SM_sensor_ouput ;  


            ////////////////////////////////////////////////////////////////


          TMP_Sum +=  TMP_sensor_ouput  ;	       // 가장 최근 값 더하고  
          TMP_Sum -=  TMP_sensor_ouput_buf[ 0 ] ;   // 가장 오랜된 값 빼고 

          TMP_sensor_ouput_avg = TMP_Sum / Avg_Num ;     // 4개 이동 평균 

           //  섭씨온도 계산 : 증폭기(증폭기 이득 = Amp_Gain ) 사용했을때 
           //TMP_sensor_ouput_avg_C =   ( unsigned short) ( (unsigned long) 1250 * TMP_sensor_ouput_avg  / (256 * Amp_Gain)  )  ;    // 온도 계산 [C] 단위

           // 섭씨온도 계산 : 증폭기 사용하지 않았을때  
          TMP_sensor_ouput_avg_C =   ( unsigned short) ( (unsigned long) 1250 * TMP_sensor_ouput_avg  / 256  )  ;           // 온도 계산 [C] 단위
 

          for( i = 0; i <= (Avg_Num - 2) ; i++ )
	      {
              TMP_sensor_ouput_buf[ i ]  = TMP_sensor_ouput_buf[ i+1 ] ;
	      } 

          TMP_sensor_ouput_buf[ Avg_Num - 1 ]  = TMP_sensor_ouput ;  

	   }

       //////////////////////////////////////////////////////////////////



   }


}







void Display_Number_LCD( unsigned int num, unsigned char digit )       // 부호없는 정수형 변수를 10진수 형태로 LCD 에 디스플레이 
{

	HexToDec( num, 10); //10진수로 변환 

	if( digit == 0 )     digit = 1 ;
	if( digit > 5 )      digit = 5 ;
 
    if( digit >= 5 )     LcdPutchar( NumToAsc(cnumber[4]) );  // 10000자리 디스플레이
	
	if( digit >= 4 )     LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스플레이 

	if( digit >= 3 )     LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스플레이 

	if( digit >= 2 )     LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스플레이

	if( digit >= 1 )     LcdPutchar(NumToAsc(cnumber[0]));    //  1자리 디스플레이

}


void Display_TMP_LCD( unsigned int tp  )       // 온도를 10진수 형태로 LCD 에 디스플레이 
{

	HexToDec( tp, 10); //10진수로 변환 

 
    LcdPutchar(NumToAsc(cnumber[2]) );   // 10자리 디스플레이
	
    LcdPutchar(NumToAsc(cnumber[1]));    // 1자리 디스플레이 

    LcdPuts( ".");                       // 소숫점(.) 디스플레이 

    LcdPutchar(NumToAsc(cnumber[0]));    // 0.1 자리 디스플레이 

 

}


void HexToDec( unsigned short num, unsigned short radix) 
{
	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;

	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 

	} while(num);

} 

char NumToAsc( unsigned char Num )
{
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}



void msec_delay(unsigned int n)
{	
	for(; n>0; n--)		// 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}

void usec_delay(unsigned int n)
{	
	for(; n>0; n--)		// 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}


/////DC함수///////////

void DC_Motor_Run_Fwd_L( short duty )   // 블라인드 DC 모터 정회전 함수 
{

    if( duty > Vmax_L )     duty = Vmax_L ;

    PORTA &= ~0x01;     //  블라인드 모터 구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = duty;       //  블라인드 모터 구동신호 + 단자 : OC1A(PB5) PWM duty 설정 

}

void DC_Motor_Run_Rev_L( short duty )   // 블라인드 DC 모터 역회전 함수 
{

    if( duty > Vmax_L )     duty = Vmax_L ;

    PORTA |= 0x01;              //  블라인드 모터 구동신호 - 단자 : 5 V 인가( PA0 = 1 );  
	OCR1A = Vmax_L - duty;      //  블라인드 모터 구동신호 + 단자 : OC1A(PB5) PWM duty 설정 

}


void DC_Motor_Stop_L( void )   // 블라인드 DC 모터 정지 함수 
{

    PORTA &= ~0x01;     //  블라인드 모터 구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = 0;          //  블라인드 모터 구동신호 + 단자 : OC1A(PB5) PWM duty = 0 설정 

}



void DC_Motor_Run_Fwd_R( short duty )   // 워터펌프 DC 모터 정회전 함수 
{

    if( duty > Vmax_R )     duty = Vmax_R ;

    PORTA &= ~0x02;     //  워터펌프 구동신호 - 단자 : 0 V 인가( PA1 = 0 );  
	OCR1B = duty;       //  워터펌프 구동신호 + 단자 : OC1B(PB6) PWM duty 설정 

}

void DC_Motor_Run_Rev_R( short duty )   // 워터펌프 역회전 함수 
{

    if( duty > Vmax_R )     duty = Vmax_R ;

    PORTA |= 0x02;                //  워터펌프 구동신호 - 단자 : 5 V 인가( PA1 = 1 );  
	OCR1B = Vmax_R - duty ;       //  워터펌프 구동신호 + 단자 : OC1B(PB6) PWM duty 설정 

}


void DC_Motor_Stop_R( void )   // 워터펌프 정지 함수 
{

    PORTA &= ~0x02;     //  워터펌프 구동신호 - 단자 : 0 V 인가( PA1 = 0 );  
	OCR1B = 0;          //  워터펌프 구동신호 + 단자 : OC1B(PB6) PWM duty = 0 설정 

}


void DC_Motor_PWM_L( short Vref )   // 블라인드 DC 모터 PWM 신호 발생 함수  
{

   if ( Vref > Vmax_L )       Vref = Vmax_L ;
   else if( Vref < -Vmax_L )  Vref = -Vmax_L ;

   if( Vref > 0 )  
   {
      DC_Motor_Run_Fwd_L( Vref ) ;
   }
   else if( Vref == 0 )  
   {
      DC_Motor_Stop_L() ;
   }
   else if( Vref < 0 )  
   {
      DC_Motor_Run_Rev_L( -Vref ) ;
   }

}


void DC_Motor_PWM_R( short Vref )   // 워터펌프 PWM 신호 발생 함수  
{

   if ( Vref > Vmax_R )       Vref = Vmax_R ;
   else if( Vref < -Vmax_R )  Vref = -Vmax_R ;

   if( Vref > 0 )  
   {
      DC_Motor_Run_Fwd_R( Vref ) ;
   }
   else if( Vref == 0 )  
   {
      DC_Motor_Stop_R() ;
   }
   else if( Vref < 0 )  
   {
      DC_Motor_Run_Rev_R( -Vref ) ;
   }


}
