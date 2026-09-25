#define F_CPU 16000000UL	// Тактовая частота 16 МГц
#include <avr/io.h>			// Библиотека именований регистров ввода-вывода
#include <util/delay.h>		// Библиотека задержек
#include <avr/interrupt.h>	// Библиотека векторов прерываний
#include <stdio.h>
#include <string.h>

#define DDR_SPI DDRB	// Регистр SPI
#define SCK DDB1		// Пин SCK
#define MOSI DDB2		// Пин MOSI
#define MISO DDB3		// Пин MISO
#define SS DDB0			// Пин SS

void spi_init(){
	DDR_SPI = (1<<MOSI) | (1<<SCK) | (1<<SS);	// Настройка пинов: MOSI, SCK, SS(PB0) - на выход
	DDR_SPI &= ~(1<<MISO);						// Настройка пина: MISO - на вход
	
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<CPHA);	// Настройка SPI Control Register для работы с ацп ADS1220
}

uint8_t spi_transfer(uint8_t data) {
	SPDR = data;					// Запись данных в регистр
	while(!(SPSR & (1 << SPIF)));	// Ожидание завершения передачи
	return SPDR;					// Чтение полученного байта
}

// Высокий и низкий уровни Chip Select
#define CS_LOW()  PORTB &= ~(1 << PB0)
#define CS_HIGH() PORTB |=  (1 << PB0)

// Команды ADS1220
#define CMD_RESET  0x06
#define CMD_START  0x08
#define CMD_RDATA  0x10

void ads1220_reset() {
	CS_LOW();
	spi_transfer(CMD_RESET);
	CS_HIGH();
	_delay_ms(1); // Время на перезагрузку
}

long ads1220_read_data() {
	uint8_t b1, b2, b3;
	
	CS_LOW();
	spi_transfer(CMD_RDATA);	// Команда на чтение данных
	b1 = spi_transfer(0xFF);	// Старший байт (DATA MSB, биты 23-16)
	b2 = spi_transfer(0xFF);	// DATA (биты 15-8)
	b3 = spi_transfer(0xFF);	// Младший байт (DATA LSB, биты 7-0)
	CS_HIGH();

	// Сборка 24-битного результата
	long result = ((long)b1 << 16) | ((long)b2 << 8) | (long)b3;
	
	// Если число отрицательное (23-й бит = 1), расширяем знак до 32 бит
	if (result & 0x800000) {
		result |= 0xFF000000;
	}
	
	return result;
}

// Флаг готовности данных
volatile uint8_t data_ready_flag = 0;

void interrupt_init() {
	// Настройка пина PD0 (INT0) на вход
	DDRD &= ~(1 << DDD0);

	// EICRA — регистр управления прерываниями (для INT0-INT3)
	// ISC01 = 1, ISC00 = 0 -> Прерывание по спадающему фронту (Falling Edge)
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);

	// EIMSK — маска внешних прерываний
	// Включение INT0
	EIMSK |= (1 << INT0);

	// Глобальное разрешение прерываний
	sei();
}

// Обработчик прерывания INT0
ISR (INT0_vect) {
	data_ready_flag = 1; // Устанавливаем флаг для основного цикла
}

void ads1220_setup(){
	CS_LOW();
	
	// Запись сразу 4 регистров, начиная с 0-го
	// Команда: 0x40 (WREG) | 0x00 (Начать с рег 0) | 0x03 (Записать 4 байта)
	spi_transfer(0x43);
	
	// Reg 0: MUX=AIN0-AIN1 (0000), GAIN=1 (000), PGA=Enable (0) -> 0x00
	spi_transfer(0x00);

	// Reg 1: 20 SPS, Continuous Mode
	// DR=20SPS (000), MODE=Normal (00), CM=Continuous (1), TS=Off (0), BCS=Off (0) -> 0x04
	spi_transfer(0x04);

	// Reg 2: VREF=Internal 2.048-V (00), 50/60=default (00), PSW=default (0), IDAC=Off (000) -> 0x00
	spi_transfer(0x00);
	
	// Reg 3: IDAC1=Off (000), IDAC2=Off (000), DRDY mode (0) -> 0x00
	spi_transfer(0x00);

	CS_HIGH();
	
	// Запуск преобразования ацп после настройки
	CS_LOW();
	spi_transfer(CMD_START); // Команда START/SYNC
	CS_HIGH();
}

void watchdog_init(){
	// Настройка пина PB4 на выход
	DDRB |= (1 << DDB4);
	
	// Установка начального значения
	PORTB &= ~(1 << PB4);
}

// Функция сброса сторожевого таймера супервизора
void watchdog_reset(){
	// Инвертирование состояние пина PB4
	PORTB ^= (1 << PB4);
}

#define RS485_CONTROL_PIN   PE2
#define RS485_CONTROL_DDR   DDRE
#define RS485_CONTROL_PORT  PORTE

#define RS485_TX_ENABLE()   RS485_CONTROL_PORT |= (1 << RS485_CONTROL_PIN)
#define RS485_RX_ENABLE()   RS485_CONTROL_PORT &= ~(1 << RS485_CONTROL_PIN)

void uart0_init(unsigned int baud) {
	unsigned int ubrr = F_CPU/16/baud - 1;
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	
	// Включение приемника и передатчика UART0
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	// Настройка UART для Modbus ASCII: 7 бит данных, Even Parity, 1 Stop Bit
	UCSR0C = (1 << UPM01) | (1 << UCSZ01);
	
	// Настройка пина управления на выход
	RS485_CONTROL_DDR |= (1 << RS485_CONTROL_PIN);
	RS485_RX_ENABLE(); // Включение приема
}

void rs485_send_byte(uint8_t data) {
	RS485_TX_ENABLE();				// Включение передачи
	
	UDR0 = data;					// Загрузка данных в буфер
	
	// Ожидание завершения передачи байта
	while (!(UCSR0A & (1 << TXC0)));
	
	// Сброс флага завершения передачи вручную
	UCSR0A |= (1 << TXC0);
	
	RS485_RX_ENABLE();				// Возвращение на прием
}

// Функция для отправки строки
void rs485_send_string(char* s) {
	while (*s) {
		rs485_send_byte(*s++);
	}
}

#define MBEE_RESET_PIN  DDD1
#define MBEE_UART_RTS  DDD4

void mbee_hardware_reset() {
	// Устанавливаем пин МК в 1, чтобы "нажать" сброс (если схема инвертирующая)
	PORTD |= (1 << MBEE_RESET_PIN);
	_delay_ms(10); // Ждем время, необходимое для разряда емкостей модуля
	
	// Устанавливаем пин МК в 0, чтобы "отпустить" сброс
	PORTD &= ~(1 << MBEE_RESET_PIN);
	
	// Ждем время инициализации прошивки модуля (до 500 мс)
	_delay_ms(500);
}

void mbee_init() {
	DDRD |= (1 << MBEE_RESET_PIN);
	DDRD &= ~(1 << MBEE_UART_RTS);
	
	mbee_hardware_reset(); // Перезагрузка модуля
}

void uart1_init(unsigned int baud) {
	unsigned int ubrr = F_CPU/16/baud - 1;
	UBRR1H = (unsigned char)(ubrr >> 8);
	UBRR1L = (unsigned char)ubrr;
	
	// Настройка пина RTS на вход
	DDRD &= ~(1 << MBEE_UART_RTS);
	
	// Включение приемника и передатчика для UART1
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	// Настройка кадра (7 бит данных, Even Parity, 1 Stop Bit для Modbus ASCII)
	UCSR1C = (1 << UPM11) | (1 << UCSZ11);
}

#define MBEE_RTS_PIN   PD4
#define MBEE_RTS_PORT  PIND

bool mbee_send_byte(uint8_t data) {
	uint32_t timeout_counter;
	
	// Ждем, пока RTS станет LOW (0)
	// Если RTS высокий (1), модуль занят - ожидание в цикле
	while (MBEE_RTS_PORT & (1 << MBEE_RTS_PIN)) {
		timeout_counter++;
		if (timeout_counter > 500000) return false; // Ошибка передачи
		watchdog_reset(); // Сбрасываем сторожевой таймер супервизора пока модуль занят
	}

	// Ждем освобождения буфера передатчика UART
	while (!(UCSR1A & (1 << UDRE1)));
	
	// Отправляем байт
	UDR1 = data;
	
	return true;
}

void mbee_send_string(char* s) {
	while (*s) {
		if (mbee_send_byte(*s)) mbee_hardware_reset();
		else *s++;
	}
}

void assemble_modbus_ascii(char* buffer, uint8_t addr, uint8_t func, int32_t val) {
	uint8_t data[4];
	
	// Разбивка числа на байты
	data[0] = (uint8_t)((val >> 24) & 0xFF); // Старший байт
	data[1] = (uint8_t)((val >> 16) & 0xFF);
	data[2] = (uint8_t)((val >> 8) & 0xFF);
	data[3] = (uint8_t)(val & 0xFF);         // Младший байт

	// Вычисление LRC (контрольная сумма)
	// LRC считается от бинарных данных: Адрес + Функция + Все байты данных
	uint8_t lrc = addr + func;
	for (uint8_t i = 0; i < 4; i++) {
		lrc += data[i];
	}
	lrc = (uint8_t)(-((int8_t)lrc)); // Финальный шаг вычисления LRC

	// Формирование итоговой строки (ASCII)
	// Формат: : [Addr][Func][Data0-3][LRC] \r\n
	// %02X означает "печать в HEX, минимум 2 символа, дополнить нулем"
	sprintf(buffer, ":%02X%02X%02X%02X%02X%02X%02X\r\n",
	addr,
	func,
	data[0], data[1], data[2], data[3],
	lrc);
}

#define ADC_OFFSET 1638400L // Соответствует 4 мА (0 бар)

int32_t code2pressure(int32_t adc_result) {
	// Вычитание смещения 4 мА
	int32_t corrected = adc_result - ADC_OFFSET;
	
	// Отсечка отрицательных значений (шум)
	if (corrected < 0) corrected = 0;

	// Расчет по формуле: (corrected * 25) / 16384
	// Используем сдвиг вправо на 14 бит вместо деления
	// Результат в миллибарах
	int32_t p_mbar = (corrected * 25L) >> 14;

	return p_mbar;
}

#define BAUD 9600
#define SLAVE_ID 0x01

int main(void)
{
	watchdog_init();
	spi_init();
	interrupt_init();
	uart0_init(BAUD);
	uart1_init(BAUD);
	ads1220_setup();
	mbee_init();
	
	while(1) {
		if (data_ready_flag) {
			data_ready_flag = 0; // Сброс флага
			
			// Чтение 24 бит через SPI и усреднение данных
			long long sum = 0;
			for(int i=0; i<16; i++) {
				sum += ads1220_read_data();
			}
			long average_result = sum / 16;
			char packet[64];
			assemble_modbus_ascii(packet, SLAVE_ID, 0x03, code2pressure(average_result)); // 0x03 - функция read
			
			// Отправка в провод (RS-485)
			rs485_send_string(packet);
			
			// Отправка в радио (MBee)
			mbee_send_string(packet);
			
			watchdog_reset();
		}
	}
}



