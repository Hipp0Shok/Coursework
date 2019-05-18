#define DEBUG 3 //1 - Отладка по серийному порту 2 - проверка входного сигнала 3 - проверка режима отображения 
#define FHT_N 256 //Количество измерений для преобразования Хартли
#define LOG_OUT 1 //Логарифмический вывод
#define MIDDLE_POINT 27 //Значение смещения на постоянную величину в 6 битном режиме АЦП
#define MIN_VALUE 30 //Нижний порог выходных значений частот
#define MAX_VALUE 120 //Верхний порог
#define DIVIDER 3.56
#define NUMBER 36
#define LOW_LEVEL 2
#define LATCH 8
#define CLOCK 13
#define DATA 11
#include <avr/io.h>

#include <FHT.h>
#include "avr/interrupt.h"
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <SPI.h>

unsigned char mode = 'r';

void offCube();

void UARTInit() //Инициализация серийного порта
{
	UBRR0H = 0;
	UBRR0L = 8; //baudrate 115200
	UCSR0A = 0;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//Разрешение приёма и передачи по юарту
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);//Установка 8 бит данных и 1 стоп бита
}

void UARTSend(unsigned char c) //Посылка пакета по серийному порту
{
	while(!(UCSR0A &(1<<UDRE0))); //Ожидание пока освободится регистр передачи
	UDR0 = c;
}

unsigned char UARTGet(void) //Получение пакета по серийному порту
{
	while(!(UCSR0A & (1<<RXC0))); //Ожидание, пока поднимется флаг о наличии непрочитанных данных в регистре
	return UDR0;
}

void UARTSendString(char *s) //Отправка строки по серийному порту
{
	while(*s != 0)
	{
		UARTSend(*s++);
	}
}

void UARTSendUInt(uint16_t c) //Отправка беззнакового целого по серийному порту
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

void ADCInit(void) //Инициализация АЦП
{
	ADMUX |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0)
				|(1<<ADLAR)
				|(1<<REFS0); //Использование А7 // Vcc + cap
	ADCSRA |= (1<<ADEN) //Включение АЦП
						|(1<<ADSC) //Запуск первого преобразования
						|(1<<ADATE) //Включение автотриггера
						|(1<<ADPS2)|(1<<ADPS0); //Делитель на 32

}
uint16_t ADCCapture() //Приём одного семпла
{
	unsigned int temp = 0;
	while(!(ADCSRA &(1<<ADIF))); //Ожидание завершения преобразования
	ADCSRA |= (1<<ADIF); //Запуск нового
	temp += ADCH;
	temp >>= 2;
	return temp;
}

void captureWave(uint16_t count) //Приём массива семплов
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



void SPI_MasterInit(void) //Инициализация SPI
{
	DDRB |= ((1<<PB2)|(1<<PB3)|(1<<PB5));
	PORTB &= ~((1<<PB3)|(1<<PB5));
	PORTB |= 1<<PB2;
	SPCR |= ((1<<SPE)|(1<<MSTR));
}

void SPI_Write(uint8_t layer, uint8_t number)			//Зажжение столбца number до уровня layer	
{
	PORTB &= ~(1<<PB2);
	uint8_t mes = 0;
	if( layer == 0 ) return;
  for( uint8_t i = 0; i < layer; i++)
	{
     mes |= (1 << i);
	}
  uint8_t shift = ceil(number/8);
	for( uint8_t i = 0; i < 5; i++)
	{
		if( i == shift)
		{
			//SPI.write()
			SPDR = ~(1 << number % 8);
			while(!(SPSR & (1<<SPIF)));
		}
		else
		{
			SPDR = 0b11111111;
			while(!(SPSR & (1<<SPIF)));
		}
	}
PORTB |= (1<<PB2);
_delay_us(50);
}

void offCube(void) //Выключение куба
{
	PORTB &= ~(1<<PB2);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(0);
	PORTB |= (1<<PB2);
}

void onCube(void) //Выключение куба
{
	PORTB &= ~(1<<PB2);
	PORTB |= (1<<PB2);
}

void draw(uint8_t output[NUMBER]) //Отрисовка всего массива
{
	offCube();
  for( uint8_t i = 0; i < NUMBER; i++ )
	{
		SPI_Write(output[i], i);
	}
}

void packEqual(uint8_t output[NUMBER]) //Разбиение диапазона на равные промежутки
{
	uint8_t number = 0;
	for(uint8_t i = 0; i < NUMBER; i++)
	{
		output[i] = 0;
	}
	for(uint8_t i = 0; i < FHT_N/2; i++)
	{
			number = floor(i/DIVIDER);
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

void testDraw()
{
	uint8_t output[37];

	for(uint8_t i = 0; i < 6; i ++)
	{
		PORTB &= ~(1<<PB2);
		SPI.transfer(1<<(2+i));
		SPI.transfer(255);
		SPI.transfer(255);
		SPI.transfer(255);
		SPI.transfer(255);
		SPI.transfer(255);
		PORTB |= (1<<PB2);
		_delay_ms(500);
	}
	uint8_t mes = 1;
	for(uint8_t i = 0; i < 6;  i++)
	{
		for( uint8_t j = 0; j < 36; j++)
		{
			PORTB &= ~(1<<PB2);
			SPI.transfer(1<<(2+i));
			for(uint8_t k = 0; k < 5; k++)
			{
				if(uint8_t(j/6) == i )
					{
						SPI.transfer(1<<(j%6));
					}
				else
				{
					SPI.transfer(0);
				}
			}
			PORTB |= (1<<PB2);
			_delay_ms(500);
		}
	}

}

int main(void)
{
	uint8_t output[NUMBER];
	ADCInit();
	SPI.begin();
	UARTInit();
	#if DEBUG == 1
	UARTInit();
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
	testDraw();
	//_delay_ms(500);
}
#endif
}

ISR(USART_RX_vect)
{
	mode = UDR0;
}





