#ifndef LCDLIB_H_
#define LCDLIB_H_

//LCDコントローラの初期設定
void lcd_init(void);

//表示位置調整関数
void lcd_pos(int line, int col);

//クリア関数
void lcd_clear(void);

void lcd_str(const char *s);
void lcd_number(unsigned int value, int digit);
#endif /* LCDLIB_H_ */
