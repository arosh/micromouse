#include "lcdlib.h"
#include <inttypes.h>
#include <avr/io.h>
#include "avr_tools.h"

#define LCD_DISPLAYFUNCTION	0b00101000 // functionset | 4bitmode | 2line | 5x8dots
#define LCD_DISPLAYCONTROL	0b00001100 // displaycontrol | displayon | cursoroff | blinkoff
#define LCD_CLEARDISPLAY	0b00000001
#define LCD_DISPLAYMODE		0b00000110 // entrymodeset | entryleft | entryshiftdecrement
#define LCD_SETDDRAMADDR	0b00001000

static void write4bits(const uint8_t value);
static void send_enable_pulse();
static void command(const uint8_t value);
static void write(uint8_t value);
static void send(const uint8_t value, const uint8_t rs);

void lcd_init(void) {
	_delay_us(50000);
	cbi(PORTC, PC0); // RS = LOW
	cbi(PORTC, PC1); // E  = LOW

	write4bits(0b0011);
	_delay_us(4500);

	write4bits(0b0011);
	_delay_us(4500);

	write4bits(0b0011);
	_delay_us(150);

	write4bits(0b0010);
	
	command(LCD_DISPLAYFUNCTION);
	command(LCD_DISPLAYCONTROL);
	lcd_clear();
	command(LCD_DISPLAYMODE);
}

void lcd_pos(int line, int col) {
	uint8_t line_offsets[] = { 0x00, 0x40 };
	command(LCD_SETDDRAMADDR | (col + line_offsets[line]));
}

void lcd_clear(void) {
	command(LCD_CLEARDISPLAY);
	_delay_us(2000);
}

void lcd_str(const char *s) {
	while(*s != '\0') {
		write(*s);
		++s;
	}
}

//数値をLCDの現在のカーソル位置から順に表示する
//valueに値を、digitに表示桁数を渡す
// 例: lcd_number(321, 3) => 321
//     lcd_number(321, 2) => 21
//     lcd_number(321, 4) => 0321
void lcd_number(unsigned int value, int digit) {
  int i;
  unsigned int base = 1;
  for(i = 0; i < digit - 1; ++i) base *= 10;
  for(i = 0; i < digit; ++i) {
    write(0x30 + (value / base) % 10);
    base /= 10;
  }
}

static void write4bits(const uint8_t value) {
	PORTC = ((value << 4) & (0b11110000)) | (PORTC & (0b00001111));
	send_enable_pulse();
}

static void send_enable_pulse() {
	cbi(PORTC, PC1); // E = LOW
	_delay_us(1);
	sbi(PORTC, PC1); // E = HIGH
	_delay_us(1);
	cbi(PORTC, PC1); // E = LOW
	_delay_us(100);
}

static void command(uint8_t value) {
	send(value, 0);
}

static void write(uint8_t value) {
	send(value, 1);
}

static void send(const uint8_t value, const uint8_t rs) {
	if(rs) {
		sbi(PORTC, PC0); // RS = 1
	}
	else {
		cbi(PORTC, PC0); // RS = 0
	}
	write4bits(value >> 4);
	write4bits(value);
}
// vim: noet ts=4 sw=4 sts=0
