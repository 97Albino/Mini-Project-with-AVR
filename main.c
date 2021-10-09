#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PORT_DATA		PORTB		//데이터 핀 연결 포트
#define PORT_C0NTROL		PORTA		//제어 핀 연결 포트
#define DDR_DATA		DDRB		//데이터 핀의 데이터 방향
#define DDR_C0NTROL		DDRA		//제어 핀의 데이터 방향

#define RS_PIN			0
#define RW_PIN			1
#define E_PIN			2

#define COMMAND_CLEAR_DISPLAY		0x01
#define COMMAND_8_BIT_MODE			0x38

#define COMMAND_DISPLAY_ON_OFF_BIT		2
#define COMMAND_CURSOR_ON_OFF_BIT		1
#define COMMAND_BLINK_ON_OFF_BIT		0

void LCD_pulse_enable(void)
{
	PORT_C0NTROL |= (1<<E_PIN);
	_delay_ms(1);
	PORT_C0NTROL &= ~(1<<E_PIN);
	_delay_ms(1);
}

void LCD_write_data(uint8_t data)
{
	PORT_C0NTROL |= (1<<RS_PIN);
	PORT_DATA = data;
	LCD_pulse_enable();
	_delay_ms(2);
}

void LCD_write_command(uint8_t command)
{
	PORT_C0NTROL &= ~(1<<RS_PIN);
	PORT_DATA = command;
	LCD_pulse_enable();
	_delay_ms(2);
}

void LCD_clear(void)
{
	LCD_write_command(COMMAND_CLEAR_DISPLAY);
	_delay_ms(2);
}

void LCD_init(void)
{
	_delay_ms(50);
	
	DDR_DATA = 0xFF;
	DDR_C0NTROL = 0x00;
	DDR_C0NTROL  |= (1 << RS_PIN) | (1 << RW_PIN) | (1<<E_PIN);
	
	PORT_C0NTROL &= ~(1 << RW_PIN);
	
	LCD_write_command(COMMAND_8_BIT_MODE);
	
	uint8_t command = 0x08 | (1<< COMMAND_DISPLAY_ON_OFF_BIT);
	LCD_write_command(command);
	
	LCD_clear();
	
	LCD_write_command(0x06);
}

void LCD_write_string(char *string)
{
	uint8_t i;
	for(i=0; string[i]; i++)
	LCD_write_data(string[i]);
}

void LCD_goto_XY(uint8_t row, uint8_t col)
{
	col %= 16;
	row %= 2;
	
	uint8_t address = (0x40 * row) + col;
	uint8_t command = 0x80 + address;
	
	LCD_write_command(command);
}


void UART1_init(void)
{
	UCSR1A |= (1 << U2X1); // 2배속 모드
	UCSR1B |= (1 << RXEN1) | (1 << TXEN1); // 송수신 활성화
	UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10); // 8비트
	UBRR1H = 0;
	UBRR1L = 207; // 보율 9600
}

unsigned char UART1_receive(void)
{
	while( !(UCSR1A & (1 << RXC1))); // 데이터 수신 대기
	
	return UDR1;
}

int level;
int mode=0;
int reverse=0;

int level;
int level_init=0;
int level_gap;
int level_UART;


uint8_t step_data[]={0x01, 0x02, 0x04, 0x08};
int step_index=-1;

uint8_t stepForward(void)
{
	step_index++;
	if(step_index>=4) step_index=0;
	
	return step_data[step_index];
}

uint8_t stepBackward(void)
{
	step_index--;
	if(step_index<0) step_index=3;
	
	return step_data[step_index];
}

void movement(void)
{
	level_gap = level-level_init;
	level_init=level;
	
	
	if(level_gap >0)
	{
		for(level_gap; level_gap >0; level_gap --)
		{
			for(int i=0; i<1000; i++)
			{
				PORTC=stepBackward();
				_delay_ms(5);
			}
		}
	}
	
	else
	{
		for(level_gap; level_gap <0; level_gap ++)
		{
			for(int i=0; i<1000; i++)
			{
				PORTC=stepForward();
				_delay_ms(5);
			}
		}
	}
	_delay_ms(200);
}

void LEVEL0(void)
{
	PORTE = 0b00000000;
	level=0;
	LCD_goto_XY(1,14);
	LCD_write_string("0 ");
	
}

void LEVEL1(void)
{
	PORTE = 0b00000001;
	level=1;
	LCD_goto_XY(1,14);
	LCD_write_string("1 ");
	
}

void LEVEL2(void)
{
	PORTE = 0b00000011;
	level=2;
	LCD_goto_XY(1,14);
	LCD_write_string("2 ");
	
}

void LEVEL3(void)
{
	PORTE = 0b00000111;
	level=3;
	LCD_goto_XY(1,14);
	LCD_write_string("3 ");
	
}

void LEVEL4(void)
{
	PORTE = 0b00001111;
	level=4;
	LCD_goto_XY(1,14);
	LCD_write_string("4 ");
	
}

void LEVEL5(void)
{
	PORTE = 0b00011111;
	level=5;
	LCD_goto_XY(1,14);
	LCD_write_string("5 ");
	
}

void LEVEL6(void)
{
	PORTE = 0b00111111;
	level=6;
	LCD_goto_XY(1,14);
	LCD_write_string("6 ");
	
}

void LEVEL7(void)
{
	PORTE = 0b01111111;
	level=7;
	LCD_goto_XY(1,14);
	LCD_write_string("7 ");
	
}

void LEVEL8(void)
{
	PORTE = 0b11111111;
	level=8;
	LCD_goto_XY(1,14);
	LCD_write_string("8 ");
	
}

void LEVEL9(void)
{
	PORTE = 0b11111110;
	level=9;
	LCD_goto_XY(1,14);
	LCD_write_string("9 ");
	
}

void LEVEL10(void)
{
	PORTE = 0b11111100;
	level=10;
	LCD_goto_XY(1,14);
	LCD_write_string("10");
	
}

int read_ADC(void)
{
	while( ! ( ADCSRA & (1 << ADIF) ) );
	return ADC;
}

ISR(USART1_RX_vect)	//UART1 수신시 인터럽트 발생
{
	
	char temp = UDR1;
	
	if(temp=='0')
	{
		mode=0;
	}
	
	else if(temp=='1')
	{
		mode=1;
	}
	
	else if(temp=='3')
	{
		reverse=1;		//리버스 출력
	}

	else if(temp=='2')
	{
		reverse=0;
	}
	
	else if(temp=='a')
	{
		level_UART=0;
	}
	
	else if(temp=='b')
	{
		level_UART=1;
	}
	
	else if(temp=='c')
	{
		level_UART=2;
	}
	
	else if(temp=='d')
	{
		level_UART=3;
	}
	
	else if(temp=='e')
	{
		level_UART=4;
	}
	
	else if(temp=='f')
	{
		level_UART=5;
	}
	
	else if(temp=='g')
	{
		level_UART=6;
	}
	
	else if(temp=='h')
	{
		level_UART=7;
	}
	
	else if(temp=='i')
	{
		level_UART=8;
	}
	
	else if(temp=='j')
	{
		level_UART=9;
	}
	
	else if(temp=='k')
	{
		level_UART=10;
	}
}

int main(void)
{
	/* Replace with your application code */
	
	LCD_clear();
	
	UCSR1B=(1<<RXCIE1);	//수신 완료 인터럽트 발생 허용 레지스터
	sei();
	
	int read;
	

	DDRE = 0xFF;
	DDRC = 0x0F;
	
	LCD_init();
	LCD_write_string("Device ON");
	LCD_goto_XY(1,0);
	LCD_write_string("Please Wait...");
	_delay_ms(3000);
	
	LCD_clear();
	
	ADMUX |= (1 << REFS0) | (1 << MUX0 | 1 << MUX1);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADFR);
	ADCSRA |= (1 << ADSC);
	
	UART1_init();
	
	LCD_write_string("MODE:Auto");
	LCD_goto_XY(1,0);
	LCD_write_string("REV:OFF LEVEL 0");
	
	while (1)
	{
		if(mode==0)
		{
			LCD_goto_XY(0,5);
			LCD_write_string("Auto  ");
		}
		
		else if (mode==1)
		{
			LCD_goto_XY(0,5);
			LCD_write_string("Manual");
		}
		
		while(mode==0)
		{
			
			while(reverse==0)
			{
				LCD_goto_XY(1,4);
				LCD_write_string("OFF");
				
				read = read_ADC();
				
				
				switch(read/100)
				{
					
					case 10 : LEVEL10(); break;
					case 9 :LEVEL9(); break;
					case 8 :LEVEL8(); break;
					case 7 :LEVEL7(); break;
					case 6 :LEVEL6(); break;
					case 5 :LEVEL5(); break;
					case 4 :LEVEL4(); break;
					case 3 :LEVEL3(); break;
					case 2 :LEVEL2(); break;
					case 1 :LEVEL1(); break;
					default : LEVEL0(); break;
				}
				_delay_ms(200);
				
				movement();
				
				if((reverse==1)||(mode==1))
				break;

			}
			
			while(reverse==1)
			{
				LCD_goto_XY(1,4);
				LCD_write_string("ON ");
				
				read = read_ADC();
				
				
				switch(read/80)
				{
					
					default : LEVEL10(); break;
					case 1 : LEVEL9(); break;
					case 2 : LEVEL8(); break;
					case 3 : LEVEL7(); break;
					case 4 : LEVEL6(); break;
					case 5 : LEVEL5(); break;
					case 6 : LEVEL4(); break;
					case 7 : LEVEL3(); break;
					case 8 : LEVEL2(); break;
					case 9 : LEVEL1(); break;
					case 10 : LEVEL0(); break;
				}
				_delay_ms(200);
				
				movement();
				
				if((reverse==0)||(mode==1))
				break;

			}
			
			if(mode==1)	//mode 1일 경우 루프 탈출
			{
				LCD_goto_XY(0,5);
				LCD_write_string("Manual");
				
				PORTE = 0b10101010;
				_delay_ms(200);
				PORTE = 0b01010101;
				_delay_ms(200);
				PORTE = 0b10101010;
				_delay_ms(200);
				PORTE = 0b01010101;
				_delay_ms(200);
				
				level_UART=level;
				
				break;
			}
			
		}

		while(mode==1)
		{
			
			if(level_UART==0)
			{
				LEVEL0();
				
				movement();
			}
			
			else if(level_UART==1)
			{
				LEVEL1();
				
				movement();
			}
			
			else if(level_UART==2)
			{
				LEVEL2();
				
				movement();
			}
			
			else if(level_UART==3)
			{
				LEVEL3();
				
				movement();
			}
			
			else if(level_UART==4)
			{
				LEVEL4();
				
				movement();
			}
			
			else if(level_UART==5)
			{
				LEVEL5();
				
				movement();
			}
			
			else if(level_UART==6)
			{
				LEVEL6();
				
				movement();
			}
			
			else if(level_UART==7)
			{
				LEVEL7();
				
				movement();
			}
			
			else if(level_UART==8)
			{
				LEVEL8();
				
				movement();
			}
			
			else if(level_UART==9)
			{
				LEVEL9();
				
				movement();
			}
			
			else if(level_UART==10)
			{
				LEVEL10();
				
				movement();
			}
			
			if(mode==0)	//mode 0일 경우 루프 탈출
			{
				LCD_goto_XY(0,5);
				LCD_write_string("Auto  ");
				
				PORTE = 0b10101010;
				_delay_ms(200);
				PORTE = 0b01010101;
				_delay_ms(200);
				PORTE = 0b10101010;
				_delay_ms(200);
				PORTE = 0b01010101;
				_delay_ms(200);
				
				break;
			}
			
		}
	}
	return 0;
}

