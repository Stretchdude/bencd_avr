/*
 * spi_slave.c
 *
 *  Created on: 09.11.2014
 *      Author: ben
 */

#include <avr/interrupt.h>  //dient zur Behandlung der Interrupts
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h> 	    //definiert den Datentyp uint8_t
#include <stdio.h>

#include <string.h>
#include "fifo.h"

#include "lcd-routines.h"

/*****************************************
 *  DEFINES
 *****************************************/

#define TRANS (1 << 0)

#define RESP_READY 0x55
#define RESP_BUSY 0x0F
#define RESP_ERROR 0xFF

#define LCDLINE 16
#define LCDNUMBEROFLINES 4
#define LCDROLL 110

#define LED_YE1   PD3
#define LED_YE2   PD4
#define LED_RED   PD5
#define LED_GREEN PD6
//#define TIMER_8BIT // 16 bit is cooler

/******************************************
 *  TYPES
 ******************************************/

enum lcd_mode {
	STATIC, ROLLING, SCROLLIN,
};

struct lcd_line {
	uint8_t fresh;
	enum lcd_mode mode;
	int8_t pos;
	uint8_t len;
	int8_t line[LCDROLL + 1];
	uint8_t rubbish[4]; // fieser pfusch. aber irgendwer schreibt aus "line" raus in fresh vom nächsten... jetzt schreibt er hier rein.
};
struct lcd_short_line {
	uint8_t fresh;
	enum lcd_mode mode;
	int8_t pos;
	int8_t line[LCDLINE + 1];
	uint8_t rubbish[4];  // :-!
};

struct lcdmodule {
	unsigned char rs; // Which GPIO pin is Register Select on the LCD *module?
	unsigned char en; // Which GPIO pin is Enable?
	unsigned char datapins[4]; // List of GPIO pins corresponding to D4, D5, D6, D7.
	unsigned int delay;
	struct lcd_line *lines[LCDNUMBEROFLINES];

};

enum frame {
	START_OF_FRAME = 0x1, START_OF_DATA = 0x2, END_OF_FRAME = 0x3
};

struct lcd_frame_data {
	uint8_t opcode;
	uint8_t len;
	uint8_t data[128];
};

/*************************************
 *  GLOBALS
 *************************************/
uint8_t state;

uint8_t gresp;
uint8_t gflag;

FIFO128_t fifo;

uint8_t read_flag = 0;
uint8_t busy_flag = 0;
uint8_t err_flag = 0;
uint8_t frame_flag = 0;

uint8_t half_sec_flag = 1;
uint8_t sec_flag = 1;
uint8_t double_sec_flag = 1;

struct lcd_line line1;
struct lcd_line line2;
struct lcd_short_line line3;
struct lcd_short_line line4;
struct lcdmodule lcd;

struct lcd_frame_data recv_frame;

uint8_t mach = 0;

static void lcd_output_line(struct lcdmodule *lcd, int8_t line_idx);
/**********************************
 * INTERRUPT SERVICE ROUTINES
 **********************************/
uint8_t cnt = 0;
ISR(SPI_STC_vect)
{
	// SPI needs to have highest ISR prio, so disable
	// timers as long we are here havin fun
	int8_t data = 0;
	uint8_t regs = SREG;
	cli();
	data = SPDR;
#if 0
	if (err_flag)
	SPDR = RESP_ERROR;
	else if (busy_flag)
	SPDR = RESP_BUSY;
	else if (read_flag)
	SPDR = RESP_READY;

//	SPDR = RESP_READY;

	//SPCR |= (1 << SPIE);

#endif
	FIFO128_write(fifo, data);

	SPDR = FIFO_look(fifo, 128);

	SREG = regs;
	SREG = regs;
	//uart_putc(data);

	if (mach == 0x00 && data == 0x01)
		mach = 0x01;
	if (mach == 0x01 && data == 0x02)
		mach = 0x02;
	if (mach == 0x02 && data == 0x02)
		mach = 0x02;
	if (mach == 0x02 && data == 0x03) {
		mach = 0x03;
		frame_flag++;
	}
	if (mach == 0x03 && data == 0x01)
		mach = 0x01;

	read_flag = 0;
}

ISR(TIMER1_COMPA_vect)
{

#define TIMER_FLAGS_USED
	static uint8_t cnt = 0;

	half_sec_flag++;
	if (cnt % 2 == 0)
		sec_flag = 1;
	if (cnt % 4 == 0)
		double_sec_flag = 1;
	if (cnt++ >= 20) {
		//line.mode = RUNNING
		// läuft ein mal durch und setzt sich zurück
		// zum augenausruhen...
	}
}

/*************************************
 *   FUNCTIONS
 **************************************/

void SPI_SlaveInit(void)
{
//DDRB |= (1 << PORTB4); //Set MISO to output
	DDRB = (1 << PB6); //Set MISO to output
//DDRB &= ~(1<<DDB5) | ~(1<<DDB7);
	SPCR = (1 << SPE) | (1 << SPIE);
	SPDR = 0;

//DDRB = (1 << PB6);	//Set MISO to output
//SPDR = RESP_READY;
}

int data_on_arival(int ms)
{
	int w = ms;
	PORTD |= (1 << LED_GREEN);

	while (0 < w-- && !FIFO_available(fifo)) {
		_delay_ms(1);
	}
	PORTD &= ~(1 << LED_GREEN);
	return (0 < w);
}

void lcd_get_frame()
{
	uint8_t len = 0;
	uint8_t tmp = 0;
	uint8_t cnt = 0;
	uint8_t ll = LCDLINE;

	// read until a valid frame starts
	tmp = FIFO128_read(fifo);
	while (START_OF_FRAME != tmp) {
		if (!data_on_arival(50)) {
			char lala[32] = { 0 };

			lcd_setcursor(1, 1);
			sprintf(lala, "##E0 0x%X (%d) %d##", tmp, FIFO_level(fifo, 128), mach);
			lcd_string(lala);
			// no valid frame in buffer
			PORTD |= (1 << LED_RED);
			err_flag = 1;
			return;
		}
		tmp = FIFO128_read(fifo);
	}
	// throw away
	if (!data_on_arival(40))
		return;

// ...protocol says there is opcode and length
	recv_frame.opcode = FIFO128_read(fifo);
	if (!data_on_arival(30))
		return;

	recv_frame.len = FIFO128_read(fifo);
// ...than START_OF_DATA flag
	if (!data_on_arival(20))
		return;

	tmp = FIFO128_read(fifo);
	if (START_OF_DATA != tmp) {
		char err[16] = "";
		sprintf(err, "##E1 0x%X (%d) %d##", tmp, FIFO_level(fifo, 128), mach); //: 0x%X%X%X%X", tmp, FIFO64_read(fifo), FIFO64_read(fifo), FIFO64_read(fifo));
		err_flag = 1;
		lcd_setcursor(1, 1);

		lcd_string(err);
		PORTD |= (1 << LED_RED);
		return;
	}
// ...now read in all the tasty new data charakters until frame is done
	while ((tmp = FIFO128_read(fifo)) != END_OF_FRAME && cnt <= recv_frame.len) {
		recv_frame.data[len++] = tmp;
		cnt++;
	}
	frame_flag--;

// ...sort out bad frame data
	switch (recv_frame.opcode) {

	case 1:
		lcd_init();
		/* no break */
	case 2:
		ll = LCDROLL;

		if (cnt > LCDLINE) {
			lcd.lines[recv_frame.opcode - 1]->mode = ROLLING;
			PORTD |= (1 << LED_YE1);
		} else {
			lcd.lines[recv_frame.opcode - 1]->mode = STATIC;
			PORTD &= ~(1 << LED_YE1);
		}

		/* no break */
	case 3:
		/* no break */
	case 4:
		if (ll < cnt)
			cnt = ll;
		memset(lcd.lines[recv_frame.opcode - 1]->line, recv_frame.opcode + 0x40, ll);

		strncpy((char *) lcd.lines[recv_frame.opcode - 1]->line, (const char *) recv_frame.data, cnt);
		lcd.lines[recv_frame.opcode - 1]->len = cnt;
		lcd.lines[recv_frame.opcode - 1]->line[cnt] = 0;
		lcd.lines[recv_frame.opcode - 1]->fresh = 1;

		err_flag = 0;
		break;
	default:
		lcd_setcursor(1, 2);
		lcd_string("##E3##");
		PORTD |= (1 << LED_RED);
		return;
	}
	read_flag = 1;

	if (!err_flag)
		PORTD &= ~(1 << LED_RED);
}

//int8_t *lcd_get_pos(struct lcd_line **pline)
int8_t *lcd_get_pos(struct lcd_line *line)
{
//	struct lcd_line *line = *pline;
	int8_t *start = NULL;
	//int16_t len = strlen((char *) line->line);
	uint8_t len = line->len;

	if (len <= line->pos) {
		line->pos = 0;
	}

	if (line->mode == STATIC)
		return line->line;
	PORTD ^= (1 << LED_YE2);
	start = line->line + line->pos;

	if (len - (LCDLINE / 2) < line->pos)
		line->pos = 0;
	else
		(line->pos)++;
	return start;
}

static void lcd_output_line(struct lcdmodule *lcd, int8_t line_idx)
{
	if (LCDNUMBEROFLINES < line_idx) {
		return;
	}

	lcd_setcursor(0, line_idx);
	prints_len((unsigned char *) lcd_get_pos(lcd->lines[line_idx - 1]), LCDLINE);

//prints_len((unsigned char *) lcd_get_pos(&(lcd->lines[line_idx])), LCDLINE);
}

void blink(int led)
{
	PORTD ^= (1 << led);
	_delay_ms(200);
	PORTD ^= (1 << led);
}

int main(void)
{

	lcd.lines[0] = &line1;
	lcd.lines[1] = &line2;
//	// this is kind of object oriented software development ;-)
	lcd.lines[2] = (struct lcd_line *) &line3;
	lcd.lines[3] = (struct lcd_line *) &line4;

	DDRD |= (1 << LED_GREEN) | (1 << LED_RED) | (1 << LED_YE2) | (1 << LED_YE1);

// timer
	// 16 bit overflow
	//TCCR1B |= (1 << CS12); //| (1 << CS10);
	//TIMSK |= (1 << TOIE1);

	// compare
	TCCR1A = 0; // this seams to be okay if cleaned
	TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12); // prescaler 1024 and CTC mode
	// 16MHz / 1024 = 15625 => 15624 = once a sec ( == 0x3D08)
	// => we like twice a second -> delete highest bit for half
	// interrupt needs to be disabled -> not enabled yet-> no problem
#ifdef TIMER_FLAGS_USED
	OCR1A = 7812;	//  15624:2; // <- 16MHz // für 14,31MHz: 0x3694; //0x1694;
#else
			OCR1A = 2048; // round about 10 times a sec
#endif
	// ignore horrorstorys about "write with a tmp register bla" -> compiler cares :-)
	//OCR1AH = (0x1694 >> 8);
	//OCR1AL = (0x1694 & 0x00FF);

	TIMSK = (1 << OCIE1A); // | (1 << OCIE1B);

	memset(fifo._buffer, 0x00, sizeof(fifo._buffer));
	FIFO_init(fifo);
	SPI_SlaveInit();

	lcd_init();
	//lcd_home();
	lcd_setcursor(0, 1);

	_delay_ms(2000);
	blink(LED_GREEN);
	sei();

#if 1
// this is not allowd any longer!	memset(lcd.lines, 0x00, sizeof(lcd.lines));
	strcpy((char *) lcd.lines[0]->line, "funktioniert denn auch der rollende Text? Auch mit langem Text?");
	strcpy((char *) lcd.lines[1]->line, "dies ist der initiale Inhalt des SPI buffers");
	strcpy((char *) lcd.lines[2]->line, "3123456789ABCDEF");
	strcpy((char *) lcd.lines[3]->line, "4FEDCBA987654321");
	lcd.lines[0]->mode = STATIC;
	lcd.lines[1]->mode = ROLLING;
#endif
	lcd.lines[2]->mode = STATIC;
	lcd.lines[3]->mode = STATIC;

	while (1) {

		if (0 < frame_flag) {
			//	uart_puts("fifoavail ");
			lcd_get_frame();
		}
#ifdef TIMER_FLAGS_USED
		if (half_sec_flag) {
			//PORTD |= (1 << LED_YE2);
			// LINE 1 of display
			//if (lcd.lines[0]->mode == ROLLING || lcd.lines[0]->fresh) {
			lcd_output_line(&lcd, 1);
			lcd.lines[0]->fresh = 0;
			//}

			// LINE 2 of display
			//if (lcd.lines[1]->mode == ROLLING || lcd.lines[1]->fresh) {
			lcd_output_line(&lcd, 2);
			lcd.lines[1]->fresh = 0;
			//}

			if (lcd.lines[2]->mode == ROLLING || lcd.lines[2]->fresh) {
				lcd_output_line(&lcd, 3);
				lcd.lines[2]->fresh = 0;
			}

			if (lcd.lines[3]->mode == ROLLING || lcd.lines[3]->fresh) {
				lcd_output_line(&lcd, 4);
				lcd.lines[3]->fresh = 0;
			}
			half_sec_flag = 0;
			//PORTD &= ~(1 << LED_YE2);
		}
#endif
	}
}

