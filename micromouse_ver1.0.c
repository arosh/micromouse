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

//各センサの値をLCDに表示
void Print_ADC(void);

//timer1のレジスタ設定
void Inti_Timer1(void);

//LCD表示のために取得値を桁ごとに分割する関数
void digit_partition(void);

void encoder(void);

//各センサ値を格納する変数
volatile unsigned char Left_Sensor_val;
volatile unsigned char LeftFront_Sensor_val;
volatile unsigned char RightFront_Sensor_val;
volatile unsigned char Right_Sensor_val;

struct{
	
	volatile unsigned char dig100;
	volatile unsigned char dig10;
	volatile unsigned char dig1;
	
}Left = {0, 0, 0}, LeftFront = {0, 0, 0}, RightFront ={0, 0, 0}, Right ={0, 0, 0};


//ロータリーエンコーダの値を格納する変数
volatile unsigned int Left_RotaryEncorder_val;
volatile unsigned int Right_RotaryEncorder_val;

struct{
	
	
	
	
	}; 



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
	 * 0: 右前センサADC入力
	 * 1: 右センサADC入力
	 * 2: 左センサADC入力
	 * 3: 左前センサADC入力
	 *
	 * 4: 右前センサのLED吸い込み
	 * 5: 右センサのLED吸い込み 
	 * 6: 左センサのLED吸い込み
	 * 7: 左前センサのLED吸い込み
	 *
	 */
	DDRA  = 0b11110000;		//念のため…リトルエンディアンです
	PORTA = 0b00000000;		//ADCで使用する際はプルアップ禁止(値が変化するため)
	
	/*
	 *	PORTB
	 *
	 * 0: スイッチ(仮)
	 * 1: スイッチ(仮)
	 * 2: スイッチ(仮)
	 * 3: 右モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 4: 右モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 5: 書き込み用ISPに使用(MOSI)
	 * 6: 書き込み用ISPに使用(MISO)
	 * 7: 書き込み用ISPに使用(SCK)
	 *
	 */
	DDRB  = 0b00011000;						
	PORTB = 0b00000111;
	
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
	PORTD = 0b00001111;
	
	//LCD初期化
	lcd_init();
	
	//タイマレジスタ設定(0:右モーターPWM 1:姿勢制御関係呼び出し 2:左モーターPWM) 
	Inti_Timer0();
	Inti_Timer2();
	
	//AD変換レジスタ設定
	Init_ADC();
	
	sei();		//割り込み許可
	
	while(1){
		
		lcd_pos(0,0);
		lcd_str("left");
		
		lcd_pos(0,5);
		
	
	}

	return 0;
}


void encoder(void)
{
	static const int dir[] = {								
		0, 1, -1, 0, -1, 0, 0, 1, 1,
		0, 0, -1, 0, -1, 1, 0
	};
	static int i;
	int n;
	
	i = (i << 2) + (PINB & 3);
	n = dir[i & 15];
	cnt += n;	//cntの値をLCDに表示
}


//各センサの値をLCDに表示
void Print_ADC(void)
{
	digit_partition();
	
	lcd_str("left front right");
	
	lcd_pos(2,1);
	lcd_data(0x30 + Left.dig100);
	lcd_data(0x30 + Left.dig10);
	lcd_data(0x30 + Left.dig1);
	
	lcd_pos(2,5);
	lcd_data(0x30 + LeftFront.dig100);
	lcd_data(0x30 + LeftFront.dig10);
	lcd_data(0x30 + LeftFront.dig1);
	
	lcd_pos(2,9);
	lcd_data(0x30 + RightFront.dig100);
	lcd_data(0x30 + RightFront.dig10);
	lcd_data(0x30 + RightFront.dig1);
	
	lcd_pos(2,13);
	lcd_data(0x30 + Right.dig100);
	lcd_data(0x30 + Right.dig10);
	lcd_data(0x30 + Right.dig1);
	
	lcd_pos(1,1);

}

//センサ値の桁をわける(LCDの文字列表示のため) 
void digit_partition(void)
{	
	//前(左側の)
	Left.dig1    =  Left_Sensor_val % 10;
	Left.dig10   = (Left_Sensor_val / 10) % 10;
	Left.dig100  = (Left_Sensor_val / 100) % 10; 

	//左のセンサ
	LeftFront.dig1    =  LeftFront_Sensor_val % 10;
	LeftFront.dig10   = (LeftFront_Sensor_val / 10) % 10;
	LeftFront.dig100  = (LeftFront_Sensor_val/ 100) % 10;

	//右のセンサ
	RightFront.dig1    =  RightFront_Sensor_val % 10;
	RightFront.dig10   = (RightFront_Sensor_val / 10) % 10;
	RightFront.dig100  = (RightFront_Sensor_val / 100) % 10;

	//前(右側)
	Right.dig1    =  Right_Sensor_val % 10;
	Right.dig10   = (Right_Sensor_val / 10) % 10;
	Right.dig100  = (Right_Sensor_val / 100) % 10;

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
	//         20MHz/1024 ==> 約19kHz
	//         #2 = 1, #1 = 0, #0 = 1
	TCCR1B = 0b0000101;
	
	//TCNT1(Timer Counter1)
	//		タイマカウンタ(8bit)に直接アクセスできる
	//		初期値をいれる
	TCNT1 = 0;
	
	
	//OCR1A(Timer Counter1 Output Compare A Register)
	//      いつコンペアマッチAをさせるかを設定する(8bit)
	//		今回は1秒カウントしたい
	//      1クロックは19kHz(5*10^-5) 今回は100msをつくる。
	//		100m/5*10^-5 = 約
	OCR1A = 5;
	
	//OCR1B(Timer Counter1 Output Compare B Register)
	//		いつコンペアマッチBをさせるかを設定する(8bit)
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

