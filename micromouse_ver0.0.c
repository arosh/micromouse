/*
 * micromouse_ver0.c
 *
 * Created: 2014/04/07 21:16:13
 *  Author: UEKI
 */ 


#include <avr/io.h>
#define F_CPU 20000000
#include <util/delay.h>
#include <avr/interrupt.h>
#include "avr_lcd.h"
#include "avr_moter.h"
#include "avr_adc.h"

//各スイッチのテスト
void switch_test(void);

//各センサの値をLCDに表示
void Print_ADC(void);

//各ロータリーエンコーダのカウント数をLCDに表示
void print_RotaryEncorder(void);

//各センサの値とロータリーエンコーダのカウント数を同時にLCDに表示
void print_all_sensor(void);

//timer1のレジスタ設定
void Inti_Timer1(void);

//timer3のレジスタ設定
void Inti_Timer3(void);

//LCD表示のためにセンサの取得値を桁ごとに分割する関数
void S_digit_partition(void);

//LCD表示のためにロータリーエンコーダの取得値を桁ごとに分割する関数
void E_digit_partition(void);

//エンコーダ設定
void encoder(void);

//Beep関数
void beep(void);

//各センサ値を格納する変数
volatile unsigned char Left_Sensor_val;
volatile unsigned char LeftFront_Sensor_val;
volatile unsigned char RightFront_Sensor_val;
volatile unsigned char Right_Sensor_val;

struct{
	
	volatile unsigned char dig100;
	volatile unsigned char dig10;
	volatile unsigned char dig1;
	
}S_Left = {0, 0, 0}, S_LeftFront = {0, 0, 0}, S_RightFront ={0, 0, 0}, S_Right ={0, 0, 0};


//ロータリーエンコーダの値を格納する変数
volatile unsigned int Left_RotaryEncorder_val;
volatile unsigned int Right_RotaryEncorder_val;

struct{
	
	volatile unsigned int dig10000;
	volatile unsigned int dig1000;
	volatile unsigned int dig100;
	volatile unsigned int dig10;
	volatile unsigned int dig1;
	
}E_Left = {0, 0, 0, 0}, E_Right = {0, 0, 0, 0};


ISR(TIMER1_COMPA_vect){
	
	Inti_ADC_get();
	
}

ISR(TIMER3_COMPA_vect){
	
	encoder();
	Inti_CW_right(50);
	Inti_CW_left(55);
	
}

int main(void)
{		
	cli();		//割り込み禁止
	
	/*
	 * 簡単なPORTの説明
	 *
	 * DDR_  方向レジスタ(0:入力 1:出力)
	 * PORT_ 出力レジスタ(入力の場合 0:プルアップ禁止 1:プルアップ有効)
	 *					 (出力の場合 0:Low 1:High)
	 *
	 */
	
	
	/*
	 *	PORTA
	 *
	 * 0: 右前センサ(右)ADC入力
	 * 1: 右センサADC入力
	 * 2: 左センサADC入力
	 * 3: 右前センサ(左)ADC入力
	 *
	 * 4: 左前センサのLED制御
	 * 5: 左センサのLED制御 
	 * 6: 右センサのLED制御
	 * 7: 右前センサのLED制御
	 *
	 */
	DDRA  = 0b11110000;
	PORTA = 0b00000000;		//ADCで使用する際はプルアップ禁止(値が変化するため)
	
	/*
	 *	PORTB
	 *
	 * 0: Beep出力
	 * 1: 橙スイッチ
	 * 2: 青スイッチ(INT2 外部割込み)
	 * 3: 右モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 4: 右モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 5: 書き込み用ISPに使用(MOSI)
	 * 6: 書き込み用ISPに使用(MISO)
	 * 7: 書き込み用ISPに使用(SCK)
	 *
	 */
	DDRB  = 0b00011001;						
	PORTB = 0b11100110;
	
	/*
	 *	PORTC
	 *
	 * 0: LCD表示用(RSの切り替え 0:コマンド 1:データ)
	 * 1: LCD表示用(Eのフラグ設定　このbitが立ちがるとLCDにデータが送信される)
	 * 2:
	 * 3:
	 * 4: LCD表示用(データバス)
	 * 5: LCD表示用(データバス)
	 * 6: LCD表示用(データバス)
	 * 7: LCD表示用(データバス)
	 *
	 */
	DDRC  = 0b11110011;					
	PORTC = 0b00000000;
	
	/*
	 *	PORTD
	 *
	 * 0: 左ロータリーエンコーダのパルス波Aを入力
	 * 1: 左ロータリーエンコーダのパルス波Bを入力
	 * 2: 右ロータリーエンコーダのパルス波Aを入力
	 * 3: 右ロータリーエンコーダのパルス波Bを入力
	 * 4: 
	 * 5: 
	 * 6: 左モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 7: 左モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 *
	 */
	DDRD  = 0b11000000;
	PORTD = 0b00001111;			//RE12Dはプルアップ不要らしいが念のためプルアップは有効に
	
	//LCD初期化
	lcd_init();
	
	//タイマレジスタ設定(0:右モーターPWM 1:センサ用 2:左モーターPWM 3:エンコーダ読み取り＋姿勢制御) 
	Inti_Timer0();
	Inti_Timer2();
	Inti_Timer1();
	Inti_Timer3();
	
	//AD変換レジスタ設定
	
	loop_until_bit_is_clear(PINB,PINB2);		//スタートスイッチ(青色)が押されるまで待機
	beep();										//ブザーを鳴らす
	
	sei();		//割り込み許可
	
	while(1){
		
		print_all_sensor();
		//print_RotaryEncorder();
		//Print_ADC();
		//switch_test();
	
	}

	return 0;
}

void beep(void)
{
	PORTB = 0b11100111;
	_delay_ms(1500);
	PORTB = 0b11100110;
}

void encoder(void)
{
	static const int dir_right[] = {								
		0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
	};
	static const int dir_left[] = {
		0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
	};
	
	static unsigned int left, right;
	unsigned int m,n;
	
	left  = (left << 2) + (PIND & 3);
	right = (right << 2) + ((PIND & 12) >> 2);
	
	m = dir_left[left & 15];
	n = dir_right[right & 15];
	
	
	Left_RotaryEncorder_val  += m;	
	Right_RotaryEncorder_val += n;

}

//各センサの値とロータリーエンコーダのカウント数を同時にLCDに表示
void print_all_sensor(void)
{
	S_digit_partition();
	E_digit_partition();
	
	lcd_pos(0,0);
	lcd_data(0x30 + E_Left.dig10000);
	lcd_data(0x30 + E_Left.dig1000);
	lcd_data(0x30 + E_Left.dig100);
	lcd_data(0x30 + E_Left.dig10);
	lcd_data(0x30 + E_Left.dig1);
	
	lcd_pos(0,6);
	lcd_data(0x30 + E_Right.dig10000);
	lcd_data(0x30 + E_Right.dig1000);
	lcd_data(0x30 + E_Right.dig100);
	lcd_data(0x30 + E_Right.dig10);
	lcd_data(0x30 + E_Right.dig1);
	
	lcd_pos(1,0);
	lcd_data(0x30 + S_RightFront.dig100);
	lcd_data(0x30 + S_RightFront.dig10);
	lcd_data(0x30 + S_RightFront.dig1);
	
	lcd_pos(1,4);
	lcd_data(0x30 + S_LeftFront.dig100);
	lcd_data(0x30 + S_LeftFront.dig10);
	lcd_data(0x30 + S_LeftFront.dig1);
	
	lcd_pos(1,8);
	lcd_data(0x30 + S_Left.dig100);
	lcd_data(0x30 + S_Left.dig10);
	lcd_data(0x30 + S_Left.dig1);
	
	lcd_pos(1,12);
	lcd_data(0x30 + S_Right.dig100);
	lcd_data(0x30 + S_Right.dig10);
	lcd_data(0x30 + S_Right.dig1);
	
	lcd_pos(0,0);
	
}

//各センサの値をLCDに表示
void Print_ADC(void)
{
	S_digit_partition();
	
	lcd_str("RF  LF  L   R");
	
	lcd_pos(1,0);
	lcd_data(0x30 + S_RightFront.dig100);
	lcd_data(0x30 + S_RightFront.dig10);
	lcd_data(0x30 + S_RightFront.dig1);
	
	lcd_pos(1,4);
	lcd_data(0x30 + S_LeftFront.dig100);
	lcd_data(0x30 + S_LeftFront.dig10);
	lcd_data(0x30 + S_LeftFront.dig1);
	
	lcd_pos(1,8);
	lcd_data(0x30 + S_Left.dig100);
	lcd_data(0x30 + S_Left.dig10);
	lcd_data(0x30 + S_Left.dig1);
	
	lcd_pos(1,12);
	lcd_data(0x30 + S_Right.dig100);
	lcd_data(0x30 + S_Right.dig10);
	lcd_data(0x30 + S_Right.dig1);
	
	lcd_pos(0,0);

}

//センサ値の桁をわける(LCDの文字列表示のため) 
void S_digit_partition(void)
{	
	//前(左側の)
	S_Left.dig1			=  Left_Sensor_val % 10;
	S_Left.dig10		= (Left_Sensor_val / 10) % 10;
	S_Left.dig100		= (Left_Sensor_val / 100) % 10; 

	//左のセンサ
	S_LeftFront.dig1    =  LeftFront_Sensor_val % 10;
	S_LeftFront.dig10   = (LeftFront_Sensor_val / 10) % 10;
	S_LeftFront.dig100	= (LeftFront_Sensor_val/ 100) % 10;

	//右のセンサ
	S_RightFront.dig1   =  RightFront_Sensor_val % 10;
	S_RightFront.dig10  = (RightFront_Sensor_val / 10) % 10;
	S_RightFront.dig100 = (RightFront_Sensor_val / 100) % 10;

	//前(右側)
	S_Right.dig1		=  Right_Sensor_val % 10;
	S_Right.dig10		= (Right_Sensor_val / 10) % 10;
	S_Right.dig100		= (Right_Sensor_val / 100) % 10;

}

void print_RotaryEncorder(void)
{
	E_digit_partition();
	
	lcd_str("rotary encorder");
	
	lcd_pos(1,0);
	lcd_data(0x30 + E_Left.dig10000);
	lcd_data(0x30 + E_Left.dig1000);
	lcd_data(0x30 + E_Left.dig100);
	lcd_data(0x30 + E_Left.dig10);
	lcd_data(0x30 + E_Left.dig1);
	
	lcd_pos(1,6);
	lcd_data(0x30 + E_Right.dig10000);
	lcd_data(0x30 + E_Right.dig1000);
	lcd_data(0x30 + E_Right.dig100);
	lcd_data(0x30 + E_Right.dig10);
	lcd_data(0x30 + E_Right.dig1);
	
	lcd_pos(0,0);
	
}

void E_digit_partition(void)
{
	//左
	E_Left.dig1			=  Left_RotaryEncorder_val % 10;
	E_Left.dig10		= (Left_RotaryEncorder_val / 10) % 10;
	E_Left.dig100		= (Left_RotaryEncorder_val / 100) % 10;
	E_Left.dig1000		= (Left_RotaryEncorder_val / 1000) % 10;
	E_Left.dig10000		= (Left_RotaryEncorder_val / 10000) % 10;
	
	//右
	E_Right.dig1		=  Right_RotaryEncorder_val % 10;
	E_Right.dig10		= (Right_RotaryEncorder_val / 10) % 10;
	E_Right.dig100		= (Right_RotaryEncorder_val / 100) % 10;
	E_Right.dig1000		= (Right_RotaryEncorder_val / 1000) % 10;
	E_Right.dig10000	= (Right_RotaryEncorder_val / 10000) % 10;
	
}

void switch_test(void)
{
	/*if(bit_is_clear(PINB,PINB1)){
		lcd_clear();
		lcd_str("orange");
		lcd_pos(0,0);
	}*/
	if(bit_is_clear(PINB,PINB2)){
		
		beep();
		
	}
	else{
		lcd_pos(0,0);
		lcd_str("Please");
		lcd_pos(1,0);
		lcd_str("press the button");
		lcd_pos(0,0);
	}
	
}

/*
 *	Function Name : Inti_Timer1
 *	Tittle        : タイマー1のレジスタ設定
 *	Input		  :	なし
 *	output        :	なし
 *	Descripution  : CTCを使って手軽にカウントする
 *					ISR(TIMER1_COMPA_vect)
 */
 
void Inti_Timer1(void)
{
	//TCCR1A(Timer Counter1 Control Register A)
	//	7,6: OC1Aから出力するPWM波の設定
	//       何も出力しないので
	//		 #7 = 0, #6 = 0
	//
	//	5,4: OC1Bから出力するPWM波の設定
	//       何も出力しないので
	//       #5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//       #3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR1Bにも設定が跨っているので注意)
	//		 CTCモード
	//       #1 = 1, #0 = 0
	TCCR1A = 0b00000010;
	
	//TCCR1B(Timer Counter1 Control register B)
	//	7,6: OC1A,OC1B 強制変更設定
	//		 これは非PWMモードを使用する際に設定する　今回は使用しない
	//		 #7 = 0, #6 = 0
	//
	//	5,4: リザーブビット
	//		 #5 = 0, #4 = 0
	//
	//	3  : PWM波形の種類の設定(上記に述べた設定の残り)
	//       #3 = 0
	//
	//	2,1,0: 分周器設定
	//         ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//         分周は1/1024
	//         20MHz/1024 ==> 約20kHz
	//         #2 = 1, #1 = 0, #0 = 1
	TCCR1B = 0b0000101;
	
	//TCNT1(Timer Counter1)
	//		タイマカウンタ(16bit)に直接アクセスできる
	//		初期値をいれる
	TCNT1 = 0;
	
	
	//OCR1A(Timer Counter1 Output Compare A Register)
	//      いつコンペアマッチAをさせるかを設定する(16bit)
	//
	//		データシートのAD変換のところを見ると、
	//		変換時間は13-260us(50k-1MHz)と書いてある。
	//		今回AD変換の動作クロックは156kHzなので線形に推移すると仮定すると約240usになる。
	//
	//		AD変換時間は約240us またマルチプレクサの切り替え時間に100usいれている。
	//		それを4回繰りかえりしているので、
	//		(240+100)*4 = 1360us ここでは1360usとする。
	//
	//		AD変換が完了する前に割り込んでも意味がないので、割り込み間隔はus以上にする必要がある。
	//
	//      1クロックは20kHz(50us)に設定しているので、
	//		3000us/50us = 60となる。
	//
	OCR1A = 2000;
	
	//OCR1B(Timer Counter1 Output Compare B Register)
	//		いつコンペアマッチBをさせるかを設定する(16bit)
	//		今回は使用しない。
	OCR1B = 0;

	//TIMSK1(Timer Counter 1 Interrupt Mask Register)
	//		タイマ割り込みを許可するためのレジスタ
	//	7,6,5,4,3: リザーブビット
	//		#7-3 = 0
	//
	//  2 : B比較の許可
	//		使用しないので
	//		#2 = 0
	//
	//  1 : A比較の許可
	//		使用するので
	//		#1 = 1
	//
	//	0 : 漏れ割り込み許可
	//		使用しないので
	//		#0 = 0
	TIMSK1 = 0b00000010;

}

void Inti_Timer3(void)
{
	//TCCR3A(Timer Counter3 Control Register A)
	//	7,6: OC3Aから出力するPWM波の設定
	//       何も出力しないので
	//			#7 = 0, #6 = 0
	//
	//	5,4: OC3Bから出力するPWM波の設定
	//       何も出力しないので
	//			#5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//			#3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR3Bにも設定が跨っているので注意)
	//			CTCモード
	//			#1 = 1, #0 = 0
	TCCR3A = 0b00000010;
	
	//TCCR3B(Timer Counter3 Control register B)
	//	7,6: OC1A,OC1B 強制変更設定
	//		 これは非PWMモードを使用する際に設定する　今回は使用しない
	//			#7 = 0, #6 = 0
	//
	//	5,4: リザーブビット
	//			#5 = 0, #4 = 0
	//
	//	3  : PWM波形の種類の設定(上記に述べた設定の残り)
	//			#3 = 0
	//
	//	2,1,0: 分周器設定
	//			ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//			ロータリーエンコーダの回転を読むので、カウントレートがサンプリング周波数よりも、
	//			大きくなってはいけないので今回のサンプリング周波数は100kHz(10us)とする
	//			20MHz/64 ==> 約312.5kHz(3.2us)
	//			#2 = 0, #1 = 1, #0 = 1
	TCCR3B = 0b0000010;
	
	//TCNT3(Timer Counter3)
	//			タイマカウンタ(16bit)に直接アクセスできる
	//			初期値をいれる
	TCNT3 = 0;
	
	
	//OCR3A(Timer Counter3 Output Compare A Register)
	//			いつコンペアマッチAをさせるかを設定する(16bit)
	//			サンプリング周波数を100kHz(10us)にしたいので
	//			10/3.2 = 3.125 ここでは約4とする
	//			
	OCR3A = 10;
	
	//OCR3B(Timer Counter3 Output Compare B Register)
	//			いつコンペアマッチBをさせるかを設定する(16bit)
	//			今回は使用しない。
	OCR3B = 0;

	//TIMSK3(Timer Counter 3 Interrupt Mask Register)
	//		タイマ割り込みを許可するためのレジスタ
	//	7,6,5,4,3: リザーブビット
	//		#7-3 = 0
	//
	//  2 : B比較の許可
	//		使用しないので
	//		#2 = 0
	//
	//  1 : A比較の許可
	//		使用するので
	//		#1 = 1
	//
	//	0 : 漏れ割り込み許可
	//		使用しないので
	//		#0 = 0
	TIMSK3 = 0b00000010;

}