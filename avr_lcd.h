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

#endif /* AVR_LCD_H_ */
