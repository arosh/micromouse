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
#include <math.h>

//各スイッチのテスト
void switch_test(void);

//各センサの値をLCDに表示
void Print_ADC(void);

//各ロータリーエンコーダのカウント数をLCDに表示
void print_RotaryEncorder(void);

//各センサの値とロータリーエンコーダのカウント数を同時にLCDに表示
void print_all_sensor(void);

//timer1のレジスタ設定
void Init_Timer1(void);

//timer3のレジスタ設定
void Init_Timer3(void);

//エンコーダ設定
void encoder(void);

//Beep関数
void beep(void);
void beep_start(void);
void beep_end(void);

//AD変換値を距離に変換
float liner_change(int x);

float liner_change(int x)
{
	return 0.0041 * x * x - 1.446 * x + 162.87;
}

//各センサ値を格納する変数
volatile unsigned char Left_Sensor_val;
volatile unsigned char LeftFront_Sensor_val;
volatile unsigned char RightFront_Sensor_val;
volatile unsigned char Right_Sensor_val;

//ロータリーエンコーダの値を格納する変数
volatile unsigned int Left_RotaryEncorder_val  = 32768;
volatile unsigned int Right_RotaryEncorder_val = 32768;

unsigned int reference_right_encoder;
unsigned int reference_left_encoder;
	
// センサ用割り込み
ISR(TIMER1_COMPA_vect){
	Init_ADC_get();
}

// エンコーダ用割り込み
ISR(TIMER3_COMPA_vect){
	
	encoder();
	char liner_front_val = (liner_change(RightFront_Sensor_val) +liner_change(LeftFront_Sensor_val)) * 0.5;
	static char turn_flag = 0;
	
	//ターンフラグが立っていないとき
	if(turn_flag == 0){
		
		//前壁がないとき
		if(liner_front_val > 59){
			
			//直進
			const float KP_RIGHT = 0.15;
			const float KP_LEFT  = 0.15;

			const char REFERENCE_RIGHT = 73;
			const char REFERENCE_LEFT  = 73;
		
			float liner_right_val;
			float liner_left_val;

			float errer_right = 0.0;
			float errer_left  = 0.0;

			float control_right = 0.0;
			float control_left  = 0.0;
		
			liner_right_val = liner_change(Right_Sensor_val);
			liner_left_val  = liner_change(Left_Sensor_val);
		
			errer_right = REFERENCE_RIGHT - liner_right_val;
			errer_left  = REFERENCE_LEFT  - liner_left_val;

			control_right = (KP_RIGHT * errer_right);
			control_left  = (KP_LEFT  * errer_left);
			if(liner_front_val < 70){
				
				Init_CCW_left(15);
				Init_CCW_right(11);
				
			}else{
				Init_CW_right((int)control_right + 45);
				Init_CW_left((int)control_left + 47);
			}
		}
		//前壁があるとき
		else{
			
			Init_CW_left(0);
			Init_CW_right(0);
			
			turn_flag = 1;
			
		}
	}
	
	//ターンフラグが立っているとき
	else{
		
		//ターンをする
		reference_right_encoder = Right_RotaryEncorder_val - 190;
		reference_left_encoder  = Left_RotaryEncorder_val + 190;
		
		const float KP_RIGHT = 0.3;
		const float KP_LEFT  = 0.32;
		
		int error_right_encoder = reference_right_encoder  - Right_RotaryEncorder_val;
		int error_left_encoder  = reference_left_encoder  - Left_RotaryEncorder_val;
		
		float control_right = KP_RIGHT * error_right_encoder;
		float control_left  = KP_LEFT  * error_left_encoder;
		
		moter_right((int)control_right - 10);
		moter_left((int)control_left + 20);
		
		//
		if( reference_left_encoder >  Left_RotaryEncorder_val){
			
			//前壁があるとき
			if(liner_front_val < 59){
				
				turn_flag = 1;
				
			}
			//前壁がないとき
			else{
				
				turn_flag = 0;
				
			}
		}
	}
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
	 * 1: LCD表示用(Eのフラグ設定 このbitが立ちがるとLCDにデータが送信される)
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
	PORTD = 0b00001111;			//RE12D(ロータリーエンコーダの名前)は
                          //プルアップ不要らしいが念のためプルアップは有効に
	
	//LCD初期化
	lcd_init();
	
	//タイマレジスタ設定
  //0:右モーターPWM
	Init_Timer0();
  //2:左モーターPWM
	Init_Timer2();
  //1:センサ用
	Init_Timer1();
  //3:エンコーダ読み取り+姿勢制御
	Init_Timer3();
	
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

void beep_start(void)
{
	PORTB = 0b11100111;
}

void beep_end(void)
{
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
  lcd_number(Left_RotaryEncorder_val, 5);
	
	lcd_pos(0,6);
  lcd_number(Right_RotaryEncorder_val, 5);

	lcd_pos(1,0);
  lcd_number(RightFront_Sensor_val, 3);
	
	lcd_pos(1,4);
  lcd_number(LeftFront_Sensor_val, 3);
	
	lcd_pos(1,8);
  lcd_number(Left_Sensor_val, 3);
	
	lcd_pos(1,12);
  lcd_number(Right_Sensor_val, 3);
	
	lcd_pos(0,0);
}

//各センサの値をLCDに表示
void Print_ADC(void)
{
	S_digit_partition();
	
	lcd_str("RF  LF  L   R");
	
	lcd_pos(1,0);
  lcd_number(RightFront_Sensor_val, 3);
	
	lcd_pos(1,4);
  lcd_number(LeftFront_Sensor_val, 3);
	
	lcd_pos(1,8);
  lcd_number(Left_Sensor_val, 3);
	
	lcd_pos(1,12);
  lcd_number(Right_Sensor_val, 3);
	
	lcd_pos(0,0);
}

void print_RotaryEncorder(void)
{
	E_digit_partition();
	
	lcd_str("rotary encorder");
	
	lcd_pos(1,0);
  lcd_number(Left_RotaryEncorder_val, 5);
	
	lcd_pos(1,6);
  lcd_number(Right_RotaryEncorder_val, 5);
	
	lcd_pos(0,0);
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
 *	Function Name : Init_Timer1
 *	Tittle        : タイマー1のレジスタ設定
 *	Input		      :	なし
 *	Output        :	なし
 *	Descripution  : CTCを使って手軽にカウントする
 *					        ISR(TIMER1_COMPA_vect)
 */
void Init_Timer1(void)
{
	//TCCR1A(Timer Counter1 Control Register A)
	//	7,6: OC1Aから出力するPWM波の設定
	//       COM1A1=0, COM1A0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		   #7 = 0, #6 = 0
	//
	//	5,4: OC1Bから出力するPWM波の設定
	//       COM1B1=0, COM1B0=0で標準ポート動作 (データシート p.83, 表16-2)
	//       #5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//       #3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR1Bにも設定が跨っているので注意)
	//		   WGM13=0, WGM12=1, WGM11=0, WGM10=0でCTCモード(データシート p.84, 表16-5の番号4)
	//       #1 = 0, #0 = 0
  // TODO 設定が間違っているのに何故か動く
	TCCR1A = 0b00000010;
	
	//TCCR1B(Timer Counter1 Control register B)
	//	7,6: ICNC1, ICES1 捕獲機道入力という謎の機能 (データシート p.82)
	//		   今回は使用しない
	//		   #7 = 0, #6 = 0
	//
	//	5: リザーブビット
	//		 #5 = 0
	//
	//	4,3: PWM波形の種類の設定(上記に述べた設定の残り)
	//       #4 = 0, #3 = 1
	//
	//	2,1,0: 分周器設定
	//         ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//         分周は1/1024
	//         20MHz/1024 ==> 約20kHz
	//         #2 = 1, #1 = 0, #0 = 1
  // TODO 設定が間違っているのに何故か動く
	TCCR1B = 0b00000101;
	
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
	//		AD変換が完了する前に割り込んでも意味がないので、割り込み間隔は1360us以上にする必要がある。
	//
	//    1クロックは20kHz(50us)に設定しているので、
	//		3000us/50us = 60となる。
	//
	OCR1A = 1500;
	
	//OCR1B(Timer Counter1 Output Compare B Register)
	//		いつコンペアマッチBをさせるかを設定する(16bit)
	//		今回は使用しない。
	OCR1B = 0;

	//TIMSK1(Timer Counter 1 Interrupt Mask Register)
	//		タイマ割り込みを許可するためのレジスタ
	//	7,6,5,4,3: リザーブビット
	//		         #7-3 = 0
	//
	//  2 : B比較の許可
	//		  使用しないので
	//		  #2 = 0
	//
	//  1 : A比較の許可
	//		  使用するので
	//		  #1 = 1
	//
	//	0 : 漏れ割り込み許可
	//		  使用しないので
	//		  #0 = 0
	TIMSK1 = 0b00000010;
}

void Init_Timer3(void)
{
	//TCCR3A(Timer Counter3 Control Register A)
	//	7,6: OC3Aから出力するPWM波の設定
	//       COM3A1=0, COM3A0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		   #7 = 0, #6 = 0
	//
	//	5,4: OC3Bから出力するPWM波の設定
	//       COM3B1=0, COM3B0=0で標準ポート動作 (データシート p.83, 表16-2)
	//       #5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//			 #3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR3Bにも設定が跨っているので注意)
	//		   WGM33=0, WGM32=1, WGM31=0, WGM30=0でCTCモード(データシート p.84, 表16-5の番号4)
	//			#1 = 0, #0 = 0
	TCCR3A = 0b00000000;
	
	//TCCR3B(Timer Counter3 Control register B)
	//	7,6: ICNC3, ICES3 捕獲機道入力という謎の機能 (データシート p.82)
	//		   今回は使用しない
	//		   #7 = 0, #6 = 0
	//
	//	5: リザーブビット
	//		 #4 = 0
	//
	//	4,3: PWM波形の種類の設定(上記に述べた設定の残り)
	//       #4 = 0, #3 = 1
	//
	//	2,1,0: 分周器設定 (データシートp.85, 表16-6)
	//			ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//			ロータリーエンコーダの回転を読むので、カウントレートがサンプリング周波数よりも、
	//			大きくなってはいけないので今回のサンプリング周波数は100kHz(10us)とする
	//			20MHz/64 ==> 約312.5kHz(3.2us)
	//			#2 = 0, #1 = 1, #0 = 1
  // TODO 設定が間違っているのに何故か動く
	TCCR3B = 0b00000010;
	
	//TCNT3(Timer Counter3)
	//			タイマカウンタ(16bit)に直接アクセスできる
	//			初期値をいれる
	TCNT3 = 0;
	
	//OCR3A(Timer Counter3 Output Compare A Register)
	//			いつコンペアマッチAをさせるかを設定する(16bit)
	//			サンプリング周波数を100kHz(10us)にしたいので
	//			10/3.2 = 3.125 ここでは約4とする
	OCR3A = 10;
	
	//OCR3B(Timer Counter3 Output Compare B Register)
	//			いつコンペアマッチBをさせるかを設定する(16bit)
	//			今回は使用しない。
	OCR3B = 0;

	//TIMSK3(Timer Counter 3 Interrupt Mask Register)
	//		タイマ割り込みを許可するためのレジスタ
	//	7,6,5,4,3: リザーブビット
	//		         #7-3 = 0
	//
	//  2 : B比較の許可
	//		  使用しないので
	//		  #2 = 0
	//
	//  1 : A比較の許可
	//		  使用するので
	//		  #1 = 1
	//
	//	0 : 漏れ割り込み許可
	//		  使用しないので
	//		  #0 = 0
	TIMSK3 = 0b00000010;
}
// vim: noet ts=4 sw=4 sts=0
