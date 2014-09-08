/*
 * avr_lcd.h
 *
 * Created: 2014/04/04 16:27:33
 *  Author: UEKI
 */

#ifndef AVR_LCD_H_
#define AVR_LCD_H_

//LCDコントローラの初期設定
void lcd_init(void);

//LCD送信設定
void lcd_out(int code, int rs);

//コマンド送信関数
void lcd_cmd(int cmd);

//データ送信関数
void lcd_data(int asci);

//表示位置調整関数
void lcd_pos(int line, int col);

//文字列送信関数
void lcd_str(char *str);

//クリア関数
void lcd_clear(void);

//数値をLCDの現在のカーソル位置から順に表示する
//valueに値を、digitに表示桁数を渡す
// 例: lcd_number(321, 3) => 321
//     lcd_number(321, 2) => 21
//     lcd_number(321, 4) => 0321
void lcd_number(long int value, int digit);

#endif /* AVR_LCD_H_ */
