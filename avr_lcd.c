/*
 * avr_lcd.c
 *
 * Created: 2014/04/04 16:51:06
 *  Author: UEKI
 */

/*
 * specification
 *
 * 詳しくは剣菱pさんの零からの電子工作第55-58回を参照されたし。
 * http://www.nicovideo.jp/watch/sm15413302 (零からの電子工作第55回リンク)
 * LCDのデータシートもそこそこ役に立つ。
 *
 * 採用したLCD
 * http://akizukidenshi.com/catalog/g/gP-02985/ (秋月電子リンク)
 *
 *
 * PORTの繋ぎ方
 *
 * 0: RS
 * 1: E
 * 2
 * 3
 * 4: DB4
 * 5: DB5
 * 6: DB6
 * 7: DB7
 *
 * PORTを節約するために8bit転送モードではなく4bit転送モードを使用している。
 * 使わないDB0-3はどことも接続しないようにしておくこと(浮かせておく)
 *
 */


#include <avr/io.h>
#define F_CPU 20000000
#include <util/delay.h>
#include "avr_lcd.h"


//文字を全消去する
void lcd_clear(void)
{
	lcd_cmd(0x01);		//クリアコマンド
}

//文字列を送信する
void lcd_str(char *str)
{
	while(*str != '\0'){
		lcd_data(*str);
		str++;
	}
}

/*文字の位置を決める(座標は0オリジン)
 *
 *	LCDは16*2
 *	(列,行)で文字の場所を決定している
 *
 *	文字列の最後は位置を(0,0)に戻してあげる(ループするために)	
 *
 */

void lcd_pos(int line, int col)
{
	if(line == 0){
		lcd_cmd(0x80 + col);
	}
	else if(line == 1){
		lcd_cmd(0xC0 + col);
	}
}

//初期設定
void lcd_init(void)
{
	_delay_ms(15);
	lcd_out(0x30, 0);
	_delay_ms(5);
	lcd_out(0x30, 0);
	_delay_ms(1);
	lcd_out(0x30, 0);
	_delay_ms(1);
	lcd_out(0x20, 0);
	_delay_ms(1);
	lcd_cmd(0x28);
	lcd_cmd(0x08);
	lcd_cmd(0x0C);
	lcd_cmd(0x06);
	lcd_cmd(0x02);
	lcd_cmd(0x01);
}

/* コマンドなのかデータを決定しLCDに送信
 *
 *	1個目の引数で値を、
 *	2個目の引数でその値がデータかコマンドかを選択する。(0:コマンド 1:データ)
 *
 */

void lcd_out(int code, int rs)
{
	PORTC = (code & 0xF0) | (PORTC & 0x0F);		//PD2,3を使う場合に値が変わらないようにするための処置
	
	if(rs == 0){
		PORTC = code & 0b11111110;				//コマンドを送信する
	}
	else{
		PORTC = code | 0b00000001;				//データを送信する
	}
	
	_delay_ms(1);
	PORTC = PORTC | 0b00000010;					//Eのフラグを立てる
	_delay_ms(1);
	PORTC = PORTC & 0b11111101;					//Eのフラグを戻す
	
}


//コマンドを送信する
void lcd_cmd(int cmd)							//4bitずつ送信
{
	lcd_out(cmd, 0);
	lcd_out(cmd << 4, 0);
	_delay_ms(2);
}

//データを送信する
void lcd_data(int asci)							//4bitずつ送信
{
	lcd_out(asci, 1);
	lcd_out(asci << 4, 1);
	_delay_ms(0.05);
}
