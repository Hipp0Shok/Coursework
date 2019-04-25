#define DEBUG 3 //1 - Отладка по серийному порту 2 - проверка входного сигнала
#define WIDTH 6 //Ширина куба/длина массива к которому приводится массив частот
#define FHT_N 256 //Количество измерений для преобразования Хартли
#define LOG_OUT 1 //Логарифмический вывод
#define OCTAVE 0 //Вывод октавами 
#define MIDDLE_POINT 25 //Значение смещения на постоянную величину в 6 битном режиме АЦП
#define MIN_VALUE 30 //Нижний порог выходных значений частот
#define MAX_VALUE 120 //Верхний порог
#define DIVIDER 3.46
#define NUMBER 36
#define LOW_LEVEL 2


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <FHT.h>
#include "avr/interrupt.h"
#include <Arduino.h>

unsigned char mode = 'r';
uint8_t intervals[7] = {0, 3, 10, 18, 36, 46, 128}; 

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
	drawClear();
}

void SPI_Write(uint8_t num)				
{
			SPDR = num;
			while(!(SPSR & (1<<SPIF)));
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

void drawClear()
{
	PORTB &= ~(1<<2);
	//SPI_Write(0);
		SPI_Write(0xFF);
		SPI_Write(0xFF);
	for(uint8_t i =0; i < 4; i++)
	{
		SPI_Write(0xFF);
	}

	PORTB |= (1<<2); 
	
}

void drawDirect(uint8_t output[NUMBER])
{
	uint8_t layer = 0, reg = 0, column = 0, mes = 0;
	for( layer = 0; layer < 6; layer++)
	{
		SPI_Write(0);
		SPI_Write(~(1 << layer));
		for( reg = 0; reg < 6; reg++)
		{
			mes = 0;
			for( column = 0; column < 6; column++)
			{
				if(reg*6 + column < NUMBER && output[reg*6 + column] >0)
				{
					mes |= (1<<column);
					output[reg*6 + column] -= 1;
				}
			}
			SPI_Write(mes);
		}
		PORTB |= (1<<2); //Обязательно разобраться с тактированием!!
		PORTB &= ~(1<<2);
		drawClear();
		PORTB |= (1<<2); //Обязательно разобраться с тактированием!!
		PORTB &= ~(1<<2);
	}
}

void drawIntervals(uint8_t output[NUMBER])
{
	uint8_t layer = 0, reg = 0, mes = 0;
	for( layer = 0; layer < 6; layer++)
	{
		PORTB &= ~(1<<2);
		SPI_Write(0);
		SPI_Write(~(1 << layer));
		for( reg = 0; reg < 6; reg++)
		{
			mes = 0;
				if(output[reg] >0)
				{
					mes = 0x3F;
					output[reg] -= 1;
				}
			SPI_Write(mes);
		}
		PORTB |= (1<<2); //Обязательно разобраться с тактированием!!
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

void drawTest()
{
	SPI_Write(0);
	SPI_Write((1 << 0));
	SPI_Write(0xFF);
	for( uint8_t i = 0; i < 4; i++)
	{
		SPI_Write(0xFF);
	}
}
int main(void)
{
	uint8_t output[NUMBER];
	//ADCInit();
	//SPI_MasterInit();
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
				//drawDirect(output);
				//drawDirect(output);
				//drawTest();
				//drawClear();
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
unsigned char i=0;
	DDRB |= ((1<<PORTB2)|(1<<PORTB3)|(1<<PORTB5)); // Ножки SPI на выход
	PORTB &= ~((1<<PORTB2)|(1<<PORTB3)|(1<<PORTB5)); //низкий уровень
	SPCR = ((1<<SPE)|(1<<MSTR)); //Включим шину, объявим ведущим
	PORTB &= ~(1<<PORTB2);
	SPDR = 0b00010000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
		SPDR = 0b00000000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
		SPDR = 0b00000000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
		SPDR = 0b00000000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
		SPDR = 0b00000000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
		SPDR = 0b00000000;
	while (!(SPSR&(1<<SPIF)));
	UARTSendString("Sent");
	PORTB |= (1<<PORTB2);
	
	_delay_ms(2000);
	
while(1)
{
	
}
#endif
}


ISR(USART_RX_vect)
{
	mode = UDR0;
}





