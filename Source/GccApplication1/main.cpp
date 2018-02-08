/*
 * GccApplication1.cpp
 *
 * Created: 16.06.2016 2:17:35
 * Author : Vitaliy
 */ 
#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned int u;
unsigned int adress=1;
unsigned int sum=0;
unsigned int rezh=0;
unsigned int k=0;

unsigned char input=0;
unsigned int send=0;
unsigned int dr=0; 

#define seg PORTB
unsigned char segmente[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF};
unsigned char N0=0;
unsigned char N1=0;
unsigned char N2=0;
unsigned char N3=0;
int number=1;

void output(unsigned int m, unsigned char m0=0, unsigned char m1=0, unsigned char m2=0, unsigned char m3=0)
{
	while(m>=1000)
	{
		m-=1000; m0=m0+1;
	}
	while(m>=100)
	{
		m-=100;  m1=m1+1;
	}
	while(m>=10)
	{
		m-=10;   m2=m2+1;
	}
	m3=m;
	
	N0=m0;
	N1=m1;
	N2=m2;
	N3=m3;
}


ISR(TIMER2_OVF_vect)
{
	PORTB = 0xFF;
	
	switch(number)
	{
		case 0:
		number=1;
		PORTC&=~(1<<PC3);
		PORTC|= (1<<PC0);
		asm volatile ("nop");
		seg=segmente[N0];
		if(adress!=4) PORTB&=~(1<<PB7);
		break;
		
		case 1:
		number=2;
		PORTC&=~(1<<PC0);
		PORTC|= (1<<PC1);
		PORTB|= (1<<PB7);
		asm volatile ("nop");
		seg=segmente[N1];
		if(adress==4) PORTB&=~(1<<PB7);
		break;
		
		case 2:
		number=3;
		PORTC&=~(1<<PC1);
		PORTC|= (1<<PC2);
		asm volatile ("nop");
		seg=segmente[N2];
		break;
		
		case 3:
		number=0;
		PORTC&=~(1<<PC2);
		PORTC|= (1<<PC3);
		asm volatile ("nop");
		seg=segmente[N3];
		if(rezh==1) PORTB&=~(1<<PB7);
		break;
	}
}

ISR(TIMER1_COMPA_vect)
{
	if(rezh==1)
	{
		adress=adress+1;
		if (adress==5) adress=1;
	}
}

int Get_ADC(void)
{
	sum=0;
	k=0;
	ADCSRA|=(1<<ADSC); // запускаємо перетворення
	while (ADCSRA & (1<<ADSC));	// чекаємо закінчення
	
	while(k!=16)
	{
	 sum=sum+ADC;
	 k=k+1;
	}
	sum=sum/k;
	_delay_ms(1);
	return sum;
}

ISR(PCINT2_vect)
{
	if(~PIND&(1<<PD7))
	{
		_delay_ms(5);
		rezh=rezh+1;
		if (rezh==2) rezh=0;
	}
	
	if(~PIND&(1<<PD6))
	{
		_delay_ms(5);
		adress=adress+1;
		if (adress==5) adress=1;
	}

}

ISR(USART_RX_vect)
{
	input=UDR0; // Прийняли символ по USART

	if(input=='c')
	{
		adress=adress+1; if (adress==5) adress=1;
	}
	if(input=='m')
	{
		rezh=rezh+1;     if (rezh==2) rezh=0;
	}
	
	if(input=='w')
	{
		send=send+1;     if (send==2) send=0;
	}
	if(input=='d')
	{
		dr=dr+1;     if (dr==2) dr=0;
	}
}

void output_usart(char data)
{
	while(!(UCSR0A&(1<<UDRE0)));   //очікування буферу для передачі
	UDR0=data; //у буфер
}

void char_output_usart(const char *string) // Якщо без const, компілятор бурчить
{
	while(*string != '\0') // Поки не дійдемо до кінця
	{
		output_usart(*string);
		string++;
	}
}

void drive(unsigned int g)
{
	g=g/4;
	char_output_usart("|");
	while(g!=0)
	{
		char_output_usart(" ");
		g=g-1;
	}
	char_output_usart(".\r\n");
}

int main(void)
{
	DDRB=0xFF;
	PORTB=0x00;
	
	DDRC=0b0001111;
	PORTC=0b1110000;
	
	DDRD=0b00111110;
	PORTB=0b11000011;
	
	PCICR|=(1<<PCIE2);
	PCIFR&=~(1<<PCIF2);
	PCMSK2|=(1<<PCINT22|1<<PCINT23);
	
	TCCR2B|=(1<<CS20|1<<CS21); //подільник на 32
	TIMSK2|=(1<<TOIE2);
	TIFR2|=(1<<TOV2);
	
	TCCR1B|=(1<<WGM12|1<<CS11|1<<CS10); //режим CTC, подільник на 32
	TCNT1=0x00;
	OCR1A=15000;
	TIMSK1|=(1<<OCIE1A);
	TIFR1|=(1<<OCF1A);
	
	ADMUX|=(1<<MUX2|1<<MUX1|1<<MUX0); //Vref зовнішнє 0.5 В
	ADCSRA|=(1<<ADEN|1<<ADPS2|1<<ADPS1|1<<ADPS0); //подільник частоти 128
	
	UBRR0L=0x0C; //12, baudrate 4800 bps
	UBRR0H=0x00;
	UCSR0B|=(1<<RXCIE0|1<<RXEN0|1<<TXEN0); //переривання по завершенню прийому
	UCSR0C|=(1<<UCSZ01|1<<UCSZ00); //посилка 8 біт
	
	sei();
	
	char_output_usart("Press [c] to switch in different channel and press [m] to switch in auto/manual mode, to write/stop [w] data with ADC\r\n");
	
	char_output_usart("channel-");
	output_usart(0x30+adress); //0x30 - код нуля по таблиці ASCII
	char_output_usart(" mode-");
	output_usart(0x30+rezh);
	char_output_usart("\r\n");

    while (1) 
    {
	
		switch(adress)
		{
						
			case 1:
			PORTD&=~(1<<PD3)&~(1<<PD4)&~(1<<PD5);
			PORTD|=(1<<PD2);
			ADMUX=(1<<MUX2)|(1<<MUX1)|(1<<MUX0);
			u=Get_ADC();
			output(u);
			break;
			
			case 2:
			PORTD&=~(1<<PD2)&~(1<<PD4)&~(1<<PD5);
			PORTD|=(1<<PD3);
			ADMUX=(1<<MUX2)|(1<<MUX1);
			ADMUX&=~(1<<MUX0);
			u=Get_ADC();
			output(u);
			break;
			
			case 3:
			PORTD&=~(1<<PD2)&~(1<<PD3)&~(1<<PD5);
			PORTD|=(1<<PD4);
			ADMUX=(1<<MUX2)|(1<<MUX0);
			ADMUX&=~(1<<MUX1);
			u=Get_ADC()*2.037;
			output(u);
			break;
			
			case 4:
			PORTD&=~(1<<PD2)&~(1<<PD3)&~(1<<PD4);
			PORTD|=(1<<PD5);
			ADMUX=(1<<MUX2);
			ADMUX&=~(1<<MUX1|1<<MUX0);
			u=Get_ADC()*2.039;
			output(u);
			break;
		}
		
		if (dr==1) drive(u);
		
		if(input=='c')
		{
			char_output_usart("channel-");
			output_usart(0x30+adress);
			char_output_usart("\r\n");
			input=0;
		}
		if(input=='m')
		{
			char_output_usart("mode-");
			output_usart(0x30+rezh);
			char_output_usart("\r\n");
			char_output_usart(" channel-");
			output_usart(0x30+adress);
			char_output_usart("\r\n");
			input=0;
		}
		if(input=='w')
		{
			char_output_usart("write ADC -");
			if(send==1) char_output_usart(" on\r\n"); else char_output_usart(" off\r\n");
			input=0;
		}
		
		if(input=='d')
		{
			char_output_usart("drive ADC -");
			if(dr==1) char_output_usart(" on\r\n"); else char_output_usart(" off\r\n");
			input=0;
		}

		if(send==1)
		{
			char_output_usart("voltage- \n");
			output_usart(0x30+adress);
			char_output_usart("  \n");
			switch(adress)
			{
				case 1:
				output_usart(0x30+N0);
				char_output_usart(".");
				output_usart(0x30+N1);
				output_usart(0x30+N2);
				output_usart(0x30+N3);
				break;
				
				case 2:
				output_usart(0x30+N0);
				char_output_usart(".");
				output_usart(0x30+N1);
				output_usart(0x30+N2);
				output_usart(0x30+N3);
				break;
				
				case 3:
				output_usart(0x30+N0);
				char_output_usart(".");
				output_usart(0x30+N1);
				output_usart(0x30+N2);
				output_usart(0x30+N3);
				break;
				
				case 4:
				output_usart(0x30+N0);
				output_usart(0x30+N1);
				char_output_usart(".");
				output_usart(0x30+N2);
				output_usart(0x30+N3);
				break;
			}
			char_output_usart(" V\r\n");
		}
		
		
	}
}