
ROS_Rx_Buf [ ]

0 ID 0x01
1 function 0x02
2 length 0x06
3 선속도 양수 0x01, 음수 0x02
///////////////////////////////////////////////////////////////
4 선속도 정수부
5 선속도 소수부

6 각속도 양수 0x01, 음수 0x02

7 각속도 정수부
8 각속도 소수부

9 CRC_L
10 CRC_R

11 주소체크 0xF0
///////////////////////////////////////////////////////////////
#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h> //strcmp 사용 sprintf
//#include "usart.h"
//#include "MPU9250.h"
#include <stdlib.h>
#include <math.h>
#define PI 3.141592
#define FUNCTION_WRITE 6
#define FUNCTION_ADDRESS 120
#define RPM_ADDRESS 121
#define FUNCTION_DATA_START 1
#define FUNCTION_DATA_STOP 2

#define RPM_MAX 2000

#define WHEEL_DIAMETER 0.125
#define WHEELBASE 0.12//0.28
#define MOTOR_REDUCER 20.0

#define LEFT_ID 2
#define RIGHT_ID 1
unsigned char BufferPacket[8] = {0,};

#define REC_BUFF_MAX_LENGTH 100
volatile unsigned char recData[REC_BUFF_MAX_LENGTH];
volatile unsigned char RecBuffindex = 0;
unsigned char RecFlg;
volatile char rec_rpm[REC_BUFF_MAX_LENGTH];
volatile char rec_angle[REC_BUFF_MAX_LENGTH];
#define REC_ANGULAR 1
#define REC_RPM 2

//double float 4byte, short,int 2byte
short rpm = 0;
short rpm_L = 0;
short rpm_R = 0;

float vel_difference = 0;
short rpm_difference = 0;
short w = 0 ;

int degree = 0;

void putchar_USART0(char data){
	while (!(UCSR0A & (1<<UDRE0))); // 송신 버퍼가 비어있을 때까지 대기
	UDR0 = data; // 데이터 전송
}

char getchar_USART0(void){
	while (!(UCSR0A & (1<<RXC0))); // 수신 완료될 때까지 대기
	return UDR0; // 수신된 데이터 반환
}
void echo_USART0(void){
	char receivedChar;
	receivedChar = getchar_USART0(); // 데이터 수신
	putchar_USART0(receivedChar); // 수신된 데이터를 다시 송신 (에코)
}


void sendBuff_USART0(char *str, int length)
{
	while (length--)
	{
		putchar_USART0(*str++); // 문자열의 끝까지 데이터를 전송
	}
}

void Init_USART0(void)
{
	UCSR0B = (1<<RXEN0) |(1<<TXEN0);
	UBRR0H = 0;
	UBRR0L = 7; //115200
	sei();
}

void Init_USART1(void)
{
	UCSR1B = (1<<RXEN1)|(1<<RXCIE1)|(1<<TXEN1);
	UBRR1H = 0;
	UBRR1L = 7; //115200
}

ISR(USART1_RX_vect)
{
	
	char receivedChar = UDR1; 
	recData[RecBuffindex++] = receivedChar; 

	if(receivedChar == ',') // 쉼표 전 linear velocity
	{
		recData[RecBuffindex-1] = '\0'; 
		strcpy(rec_rpm, recData); 
		RecBuffindex = 0; 
		memset(recData, 0, REC_BUFF_MAX_LENGTH); 
	}
	else if(receivedChar == '\n') // 쉼표 이후 개행문자 전 angular velocity
	{
		recData[RecBuffindex-1] = '\0'; 
		strcpy(rec_angle, recData);
		RecBuffindex = 0; 
		memset(recData, 0, REC_BUFF_MAX_LENGTH); 
	}
	
}
void putch_USART1(unsigned char data)
{
	while (!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;
}

void puts_USART1(unsigned char *str)
{
	unsigned char total = 8;
	while(total--){
		putch_USART1(*str);
	str++;}
}

void Comport_USART1(unsigned char *str)
{
	while(*str != 0){
		putch_USART1(*str);
	str++;}
}

unsigned short CRC16_modbus(unsigned char *addr, int num)
{
	unsigned short CRC = 0xFFFF;
	int i;
	while (num--)
	{
		CRC ^= *addr++;
		for (i = 0; i < 8; i++)
		{
			if (CRC & 1)
			{
				CRC >>= 1;
				CRC ^= 0xA001;
			}
			else
			{
				CRC >>= 1;
			}
		}
	}
	return CRC;
}
void putchar_USART1(char data)
{
	while( !(UCSR1A & (1<<UDRE1)));
	UDR1 = data;
}

void sendBuff_USART1(char *str,int length)
{  // 문자열 출력 루틴
	while (length--)
	{
		putchar_USART1(*str++);  
	}
}

/*****************************************************************************
*		: [8][8][16][16][16]
*Packet : [id][order][Fnt adress][Fnt adress][data][data][CRC16bit][CRC16bit]
******************************************************************************/
void makePacket(unsigned char id, unsigned char order, unsigned short address, short data)
{
	BufferPacket[0] = id;
	
	BufferPacket[1] = order;

	BufferPacket[2] = 0x00;
	BufferPacket[3] = address;
	
	BufferPacket[4] = data/256;
	BufferPacket[5] = data%256;
	
	if(data < 0){ --BufferPacket[4]; } //or ~ -1 
		
	unsigned short CRC = CRC16_modbus(BufferPacket, 6);
	BufferPacket[6] = CRC%256;
	BufferPacket[7] = CRC/256;
	//sendBuff_USART1(BufferPacket,sizeof(BufferPacket));
	_delay_ms(100);
}
///////////////////////////////////////////////////////////////
void RPM(unsigned char id, short rpm)//실 속도 패킷
{
	makePacket(id, FUNCTION_WRITE, RPM_ADDRESS, rpm);
	sendBuff_USART1(BufferPacket,sizeof(BufferPacket));
	_delay_ms(30);
	sendBuff_USART0(BufferPacket, sizeof(BufferPacket)); 
	
}

void Start(unsigned char id) //뒷 패킷
{
	makePacket(id, FUNCTION_WRITE, FUNCTION_ADDRESS, FUNCTION_DATA_START);//FUNCTION_ADDRESS 120 ==> HEX78 
	sendBuff_USART1(BufferPacket,sizeof(BufferPacket));
	_delay_ms(30);
	sendBuff_USART0(BufferPacket, sizeof(BufferPacket)); 
	
}

/*
void Stop(unsigned char id)
{
	makePacket(id, FUNCTION_WRITE, FUNCTION_ADDRESS, FUNCTION_DATA_STOP);
	
	sendBuff_USART0(BufferPacket, sizeof(BufferPacket)); 
	_delay_ms(10);
}
*/

void forward()
{
	w = atof(rec_angle);
	rpm = atoi(rec_rpm);
	
	vel_difference = w * WHEELBASE;
	rpm_difference = (60.0 * vel_difference * MOTOR_REDUCER) / (PI * WHEEL_DIAMETER);
	
	rpm_L = (-rpm) + rpm_difference/2;
	rpm_R = rpm + rpm_difference/2;
	
	
	if(rpm_L < -RPM_MAX)
	{
		rpm_L = -RPM_MAX;
		rpm_R = RPM_MAX + rpm_difference;
	}
	else if(rpm_R > RPM_MAX)
	{
		rpm_L = -RPM_MAX+ rpm_difference;
		rpm_R = RPM_MAX;
	}
	
	RPM(LEFT_ID, rpm_L);
	RPM(RIGHT_ID, rpm_R);
	_delay_ms(1);
	//Start(2);
	//Start(1);
}


int main(void)
{
	Init_USART1();
	Init_USART0();
	
	
	while (1)
	{
		//Stop(2);
		forward();
		_delay_ms(20);
		//echo_USART0();
		//_delay_ms(10); 
	}
}


/*



#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h> //strcmp 사용 sprintf
//#include "usart.h"
//#include "MPU9250.h"
#include <stdlib.h>
#include <math.h>
#define PI 3.141592
#define FUNCTION_WRITE 6
#define FUNCTION_ADDRESS 120
#define RPM_ADDRESS 121
#define FUNCTION_DATA_START 1
#include "usart.h"
#define FUNCTION_DATA_STOP 2

//MPU9250 SPI 통신 
//#define HIGH_SPI_CS() (PORTB |= (1 << PB0))  // SPI_CS를 HIGH로 설정
//#define LOW_SPI_CS() (PORTB &= ~(1 << PB0)) // SPI_CS를 LOW로 설정

#define REC_BUFF_MAX_LENGTH 100
/ **************variable************************* /
unsigned char RecBuff[REC_BUFF_MAX_LENGTH];
unsigned char RecBuffindex;
unsigned char Rec_flag;
unsigned char RecBuff_estimate_Length = REC_BUFF_MAX_LENGTH;

volatile unsigned char chk_Length;
volatile unsigned short chk_crc[1];
volatile unsigned char chk_rescBuff[100];
volatile unsigned char resBuffLength;
/ ***************function************************** /
void putstr_USART1(char *str);
void putchar_USART1(char data);
void sendBuff_USART1(char *str,int length);
void Init_USART1(void);
void resPacket(char addr, unsigned char func, unsigned char start_addr_1, unsigned char start_addr_2, unsigned char resData_1,unsigned char resData_2);
unsigned short CRC16bit(unsigned char *addr, int length);
int parsingPacket(unsigned char *recBuff, int length);
/ ************************************************ /

ISR(USART1_RX_vect)// 인터럽트 루틴에서의 데이터 수신
{
	
	RecBuff[RecBuffindex++] = UDR1;
	
	if(RecBuffindex == 7)
	{
		RecBuff_estimate_Length = RecBuffindex;
	}
	else;
	
	if(RecBuffindex == RecBuff_estimate_Length)
	{
		// 수신된 데이터의 순서가 패킷 길이와 같으면 패킷 수신완료
		Rec_flag = 1; // 수신 완료 플래그 활성화
		
	}
	else;
	
	putchar_USART1(UDR1);
}



ISR(USART0_RX_vect)// 인터럽트 루틴에서의 데이터 수신 
{
	putchar_USART1(UDR0); //UDR0를 UDR1으로 옮김
	//_delay_ms(100);
}


int main(void)
{
	unsigned char temp_RecBuff[100];        // 수신패킷 임시저장용 배열
	unsigned char temp_RecBuffLength = 0;   // 수신패킷 길이 저장

	int res = 0;
	sei();
	Init_USART1();
	Init_USART0();
	DDRB = 0xff;
	while(1)
	{
		
		
		if(Rec_flag == 1) // 패킷 수신완료 플래그가 설정된 경우
		{
			PORTB = 0xf0;
			// 데이터의 연속 수신을 고려하여, 임시변수에 수신된 데이터 저장
			memcpy(temp_RecBuff, RecBuff, RecBuff_estimate_Length); //memory copy 함수
			temp_RecBuffLength = RecBuff_estimate_Length;
			
			// 임시저장 후 또다시 패킷 수신을 위한 버퍼 및 관련변수 초기화
			Rec_flag = 0;
			memset(RecBuff, 0, REC_BUFF_MAX_LENGTH);
			RecBuff_estimate_Length = REC_BUFF_MAX_LENGTH;
			RecBuffindex = 0;
			
			res = parsingPacket(temp_RecBuff,temp_RecBuffLength);
			
			resBuffLength=0;
			if(res < 0)// packet 이상이 있는 경우 상태알림
			{
				char msg[20] = {0,};
				sprintf(msg, "error %d \r\n", res);
				putstr_USART1(msg);
			}
			else;
		}
	}
}


unsigned short CRC16bit(unsigned char *addr, int length)
{
	unsigned short CRC = 0xFFFF;
	
	while (length--)
	{
		CRC ^= *addr++; //XOR 같으면0 다르면1
		
		for (int i = 0; i < 8; i++)
		{
			if (CRC & 1)
			{
				CRC >>= 1; //CRC=(CRC>>1)
				CRC ^= 0xA001; //CRC= CRC^0xA001 //1010 0000 0000 0001
			}
			else
			{
				CRC >>= 1;
			}
		}
	}
	return CRC;
}



void resPacket(char addr, unsigned char func, unsigned char start_addr_1, unsigned char start_addr_2, unsigned char resData_1,unsigned char resData_2)
{ //[address,function,starting addr,data,CRC16]
	//ex) [01] _ [06] _[00},{79]_[00},{64] [59},{F8]
	unsigned char rescBuff[REC_BUFF_MAX_LENGTH];
	unsigned char resBuffLength = 0;

	rescBuff[resBuffLength++] = addr;               //address
	rescBuff[resBuffLength++] = func;               //function
	
	rescBuff[resBuffLength++] = start_addr_1;         // Starting address high byte만 추출
	rescBuff[resBuffLength++] = start_addr_2;         // Starting address low byte만 추출
	
	rescBuff[resBuffLength++] = resData_1;            //data
	rescBuff[resBuffLength++] = resData_2;            //data

	unsigned short crc = CRC16bit(rescBuff, resBuffLength);
	
	rescBuff[resBuffLength++] = (crc & 0xFF);         // CRC low "byte"
	rescBuff[resBuffLength++] = (crc >> 8);            // CRC high "byte"
	
	//resBuffLength++;
	sendBuff_USART0(rescBuff,resBuffLength); //UDR0로 data를 보냄
	crc = 0;
	PORTB = 0xaa;
	memset(rescBuff, 0, resBuffLength);
	
}

int parsingPacket(unsigned char *recBuff, int length) // 패킷을 파싱하는 함수 // 10/19 char -> unsigned char 형으로 바꿔줌
{
	

	switch(recBuff[1]) //function에 따라 case 분류 //구조: //[address,function,starting addr,data,CRC16]  //ex) [01]_[06]_[00},{79]_[00},{64]_[59},{F8]
	{
		case 0x06:  // function = 0x06인 경우  -> 드라이버의 파라미터 관련
		switch(recBuff[3])
		{
			case 0x78:
			if (recBuff[5]== 0x02)//01 06 00 78 00 02 88 12    -> stop
			{
				resPacket(0x01,0x06,0x00,0x78,0x00,0x02); //STOPID1
				_delay_ms(10);
				resPacket(0x02,0x06,0x00,0x78,0x00,0x02);//STOPID2
			}
			
			else;
			break;
			
			case 0x79:
			resPacket(recBuff[0],recBuff[1],recBuff[2],recBuff[3],recBuff[4],recBuff[5]);
			_delay_ms(10);
			if (recBuff[0] == 0x01)
			{
				resPacket(0x01,0x06,0x00,0x78,0x00,0x01);
				_delay_ms(10);
				/ **************************** /
			/ *	
				unsigned int add_data = ((unsigned int)recBuff[4] << 8) | recBuff[5];
				unsigned int negative_data = ~add_data + 1; //int -> insigned int 로 수정
				
				
				recBuff[4] = (unsigned char)(negative_data >> 8); // 상위 바이트
				recBuff[5] = (unsigned char)negative_data;        // 하위 바이트
				
				resPacket(0x02,0x06,0x00,0x79,recBuff[4],recBuff[5]);
				_delay_ms(10);
				resPacket(0x02,0x06,0x00,0x78,0x00,0x01);
				add_data =0;
				negative_data = 0;* /
			}
			else if (recBuff[0] == 0x02)
			{
				
				resPacket(0x02,0x06,0x00,0x78,0x00,0x01);
				_delay_ms(10);
			/ *	unsigned int add_data = ((unsigned int)recBuff[4] << 8) | recBuff[5];
				unsigned int negative_data = ~add_data + 1; //int -> insigned int 로 수정10/19
				
				recBuff[4] = (unsigned char)(negative_data >> 8); // 상위 바이트
				recBuff[5] = (unsigned char)negative_data;        // 하위 바이트
				
				resPacket(0x01,0x06,0x00,0x79,recBuff[4],recBuff[5]);
				_delay_ms(10);
				resPacket(0x01,0x06,0x00,0x78,0x00,0x01);
				add_data =0;
				negative_data = 0;* /
			}
			else;
			memset(recBuff, 0, length);
			break;
		}
		break;
		
		case 0x07:
		// 내용 추가 //
		break;
		
		default:
		
		break;
	}
	return 0;
}


*/
