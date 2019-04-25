#define DEBUG 1 //1 - Отладка по серийному порту 2 - проверка входного сигнала
#define WIDTH 6 //Ширина куба/длина массива к которому приводится массив частот
#define FHT_N 256 //Количество измерений для преобразования Хартли
#define LOG_OUT 1 //Логарифмический вывод
#define OCTAVE 0 //Вывод октавами 
#define MIDDLE_POINT 29 //Значение смещения на постоянную величину в 6 битном режиме АЦП
#define MIN_VALUE 30 //Нижний порог выходных значений частот
#define MAX_VALUE 120 //Верхний порог
#define DIVIDER 3.46
#define NUMBER 30
#define LOW_LEVEL 2

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <FHT.h>
#include "avr/interrupt.h"
#include <Arduino.h>

unsigned char mode = 'r';
uint8_t intervals[7] = {0, 3, 10, 18, 36, 46, 128}; 
unsigned char led_on[3]={127,127,127};
unsigned char led_off[3]={0,0,0};
unsigned char temp[NUMBER][3];
uint8_t colors[6][3] =
{
	{64, 0, 0},
	{32, 32, 0},
	{0, 64, 0},
	{0, 32, 32},
	{0, 0, 64},
	{64, 64, 64}
};

void drawClear();

void UARTInit()
{
	UBRR0H = 0;
	UBRR0L = 8; //baudrate 115200
	UCSR0A = 0;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//Разрешение приёма и передачи по юарту
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);//Установка 8 бит данных и 1 стоп бита
}

void UARTSend(unsigned char c)
{
	while(!(UCSR0A &(1<<UDRE0))); //Ожидание пока освободится регистр передачи
	UDR0 = c;
}

unsigned char UARTGet(void)
{
	while(!(UCSR0A & (1<<RXC0))); //Ожидание, пока поднимется флаг о наличии непрочитанных данных в регистре
	return UDR0;
}

void UARTSendString(char *s)
{
	while(*s != 0)
	{
		UARTSend(*s++);
	}
}

void UARTSendUInt(uint16_t c)
{
	unsigned char temp;
	c = c%10000;
	temp = c/100;
	UARTSend(temp/10+'0');
	UARTSend(temp%10+'0');
	temp=c%100;
	UARTSend(temp/10+'0');
	UARTSend(temp%10+'0');
}

void ADCInit(void)
{
	ADMUX |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0)
				|(1<<ADLAR)
				|(1<<REFS0); //Использование А7 // Vcc + cap
	ADCSRA |= (1<<ADEN) //Включение АЦП
						|(1<<ADSC) //Запуск первого преобразования
						|(1<<ADATE) //Включение автотриггера
						|(1<<ADPS2)|(1<<ADPS0); //Делитель на 32

}
uint16_t ADCCapture()
{
	unsigned int temp = 0;
	while(!(ADCSRA &(1<<ADIF))); //Ожидание завершения преобразования
	ADCSRA |= (1<<ADIF); //Запуск нового
	temp += ADCH;
	temp >>= 2;
	return temp;
}

void captureWave(uint16_t count)
{
	for(uint16_t i =0; i <count; i++)
	{
		fht_input[i] = int(ADCCapture()) - MIDDLE_POINT; // Сдвиг в ноль
		if(abs(fht_input[i]) < LOW_LEVEL)
		{
			fht_input[i] = 0;
		}
		fht_input[i]<<=10; //Приведение к 16 битному формату
	}
}

void SPI_MasterInit(void)
{
	DDRB =(1<<PB3)|(1<<PB2)|(1<<PB5)|(0<<PB4); //pb2 - ss pb3 - MOSI pb4 - MISO PB5 - SCK
	PORTB &= ~((1<<PORTB2)|(1<<PORTB3)|(1<<PORTB5));
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<CPHA);//fosc/2 16MHz/2
	SPSR |=(1<<SPI2X);
}

void SPI_Write()				
{
unsigned char a;
unsigned char j,i,n;

for (n=0; n<NUMBER; n++)
{
	for (j=0; j<3; j++)
	{
		a = 0x80;	
		for (i=0;i<8;i++)	
		{
			if ((temp[n][j]&a)==0)
			{
				SPDR = 224;//0xE0 0.37us 0	
			}
			else
			{
				SPDR = 252;//0xFC 0.75us 1 
			}
			while(!(SPSR & (1<<SPIF)));
			a=a>>1;
		}
	}
}
PORTB |= (1<<PB2);
PORTB &= ~(1<<PB2);
_delay_us(50);
}

void off_strip(void)
{
	unsigned char j,i;
			for (i=0; i<NUMBER; i++)
			{
				for (j=0;j<3;j++)
				{
					temp[i][j]=led_off[j];
				}
			}
			SPI_Write();
}



void draw(uint8_t output[NUMBER])
{
	uint8_t j = 0;
	for( uint8_t i = 0; i < NUMBER; i++)
	{
			switch (output[i])
			{
				case 0:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = led_off[j];
					}
					break;
				case 1:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[0][j];
					}
					break;
				case 2:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[1][j];
					}
					break;
				case 3:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[2][j];
					}
					break;
				case 4:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[3][j];
					}
					break;
				case 5:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[4][j];
					}
					break;
				case 6:
					for( j = 0; j < 3; j++)
					{
						temp[i][j] = colors[5][j];
					}
					break;
				default:
					break;
			}
	}
	SPI_Write();
}
void packEqual(uint8_t output[NUMBER])
{
	uint8_t number = 0;
	for(uint8_t i = 0; i < NUMBER; i++)
	{
		output[i] = 0;
	}
	for(uint8_t i = 0; i < FHT_N/2; i++)
	{
			number = ceil(i/DIVIDER);
			if(fht_log_out[i] > output[number])
			{
				output[number] = fht_log_out[i];
			}
	}
	for(uint8_t i = 0; i < NUMBER; i++)
	{
		output[i] = map(output[i], 0, MAX_VALUE, 0, 7);
		output[i] = constrain(output[i], 0, 6);
	} 	
}

void packIntervals(uint8_t output[6])
{
	for(uint8_t i = 0; i < NUMBER; i++)
	{
		output[i] = 0;
	}
	for( uint8_t i = 0; i < 6; i++)
	{
		for( uint8_t j = intervals[i]; j < intervals[i+1]; j++ )
		{
			if( fht_log_out[j] > output[i])
			{
				output[i] = fht_log_out[j];
			}
		}
		output[i] = map(output[i], 0, MAX_VALUE, 0, 7);
		output[i] = constrain(output[i], 0, 6);
	}
}

int main(void)
{
	uint8_t output[NUMBER];
	ADCInit();
	SPI_MasterInit();
	UARTInit();
	#if DEBUG == 1
	while(1)
	{
		cli();
		switch(mode)
		{
			case 'r':
				captureWave(FHT_N); //Сбор значений
				fht_window(); //Функция окна
				fht_reorder(); //Перераспределение для верной работы функции
				fht_run(); //Запуск функции Хартли
				fht_mag_log(); //Приведение к логарифмическому выводу
				fht_log_out[0] *= 0.6;
				fht_log_out[1] *= 0.3;
			for (uint16_t i = 0; i < FHT_N/2; i++)
				{
					if(fht_log_out[i]<MIN_VALUE)
					{
						fht_log_out[i] = MIN_VALUE;
					}
					fht_log_out[i]-=MIN_VALUE;
				} 
				
				sei();
				packEqual(output);
				for(uint8_t i=0; i<NUMBER; i++)
					{
						UARTSend(output[i] + '0');
						UARTSend(124);
					} 
					UARTSend(10);
				draw(output);
				break;
			case 's':
				sei();
				UARTSendString("Program is stopped by command");
				UARTSend(10);
				mode = 'w';
				break;
			default:
				_delay_ms(1);
		}
    if(UCSR0A &(1<<RXC0))
    {
      mode = UDR0;
    }
	} 
#elif DEBUG==2
while(1)
{
	switch (mode)
	{
		case 'r':
		cli();
	for(uint16_t i=0; i < 256; i++)
	{
		fht_input[i] = ADCCapture();
	} 
	//captureWave(FHT_N);
	sei();
	for(uint16_t i=0; i < 256; i++)
	{
		UARTSendUInt(abs(fht_input[i]));
		UARTSend(124);
	} 
	UARTSend(10);
	mode = 'w';
			break;
		default:
			sei();
	}
	
	
}
#elif DEBUG == 3

while(1)
{
	
}
#endif
}


ISR(USART_RX_vect)
{
	mode = UDR0;
}





