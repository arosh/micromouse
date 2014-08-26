/*
 * main.c
 *
 * Created: 2014/04/07 21:16:13
 *  Author: UEKI
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_tools.h"
#include "avr_lcd.h"
#include "avr_motor.h"
#include "avr_adc.h"
#include <math.h>

volatile int count = 0;

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

//AD変換値を距離に変換(RF)
float sensor_distance_convert_RF (int x);

//AD変換値を距離に変換(LF)
float sensor_distance_convert_LF (int x);

//AD変換値を距離に変換(L)
float sensor_distance_convert_L (int x);

//AD変換値を距離に変換(R)
float sensor_distance_convert_R (int x);

float sensor_distance_convert_RF(int x)
{
	return 0.0038 * x * x - 1.3275 * x + 141.36;
}

float sensor_distance_convert_LF(int x)
{
	return 0.0031 * x * x - 1.1545 * x + 136.38;
}

float sensor_distance_convert_L(int x)
{
	return 0.0025 * x * x - 0.9635 * x + 132.45;
}

float sensor_distance_convert_R(int x)
{
	return 0.0023 * x * x - 0.9322 * x + 133.16;
}

//各センサ値を格納する変数
extern volatile unsigned char Left_Sensor_val;
extern volatile unsigned char LeftFront_Sensor_val;
extern volatile unsigned char RightFront_Sensor_val;
extern volatile unsigned char Right_Sensor_val;

volatile float sensor_distance_LF;
volatile float sensor_distance_RF; 
volatile float sensor_distance_L;
volatile float sensor_distance_R;

//ロータリーエンコーダの値を格納する変数
volatile unsigned int Left_RotaryEncorder_val;
volatile unsigned int Right_RotaryEncorder_val;

unsigned int reference_right_encoder;
unsigned int reference_left_encoder;
	
// センサ用割り込み
ISR(TIMER1_COMPA_vect){
	Init_ADC_get();
}

// エンコーダ用割り込み
ISR(TIMER3_COMPA_vect){
	static int sensor_count = 0;
	
	if(sensor_count >= 250){
		Init_ADC_get();
		sensor_count = 0;
	}
	sensor_count++;
	
	encoder();
	
	/*//それぞれのAD変換値を距離[mm]に変換
	sensor_distance_LF = sensor_distance_convert_LF(LeftFront_Sensor_val);
	sensor_distance_RF = sensor_distance_convert_RF(RightFront_Sensor_val);
	sensor_distance_L  = sensor_distance_convert_L(Left_Sensor_val);
	sensor_distance_R  = sensor_distance_convert_R(Right_Sensor_val);
 	
	//前壁検知のためにフロントのセンサの平均値をとる
	float sensor_distance_LF_RF_AVE = (sensor_distance_LF + sensor_distance_RF) * 0.5;
	
	//エンコーダをよみとる
	encoder();
	
	static char turn_flag = 0;
	
	//ターンフラグが立っていないとき
	if(turn_flag == 0){
		
		//前壁がないとき
		if(sensor_distance_LF_RF_AVE > 60){
			
			//直進
			const float KP_RIGHT = 0.15;
			const float KP_LEFT  = 0.15;

			const char REFERENCE_RIGHT = 70;
			const char REFERENCE_LEFT  = 70;

			float errer_right = REFERENCE_RIGHT - sensor_distance_R;
			float errer_left  = REFERENCE_LEFT  - sensor_distance_L;

			float control_right = (KP_RIGHT * errer_right);
			float control_left  = (KP_LEFT  * errer_left);
			
			motor_right((int)control_right + 38);
			motor_left((int)control_left + 41);
			
		}
		//前壁があるとき
		else{
			
			motor_brake_right();
			motor_brake_left();
			
			turn_flag = 1;
			
		}
	}
	
	//ターンフラグが立っているとき
	else{
		//ターンをする
		reference_right_encoder = Right_RotaryEncorder_val - 190;
		reference_left_encoder  = Left_RotaryEncorder_val + 190;
		
		const float KP_RIGHT = 0.20;
		const float KP_LEFT  = 0.20;
		
		int error_right_encoder = reference_right_encoder  - Right_RotaryEncorder_val;
		int error_left_encoder  = reference_left_encoder  - Left_RotaryEncorder_val;
		
		float control_right = KP_RIGHT * error_right_encoder;
		float control_left  = KP_LEFT  * error_left_encoder;
		
		motor_right((int)control_right - 10);
		motor_left((int)control_left + 20);
		
		//
		if( reference_left_encoder >  Left_RotaryEncorder_val){
			//前壁があるとき
			if(sensor_distance_LF_RF_AVE < 60){
				turn_flag = 1;
			}
			//前壁がないとき
			else{
				turn_flag = 0;
			}
		}
	}
	*/
}

int main(void)
{		
	cli();		//割り込み禁止
	
	/*
	 * 簡単なPORTの説明(x:アルファベットABCD n:数字0123...)
	 *
	 * DDRx  方向レジスタ(0:入力 1:出力)
	 * PORTx 出力レジスタ(入力の場合 0:プルアップ禁止 1:プルアップ有効)
	 *					 (出力の場合 0:Low 1:High)
	 * PINx  入力レジスタ
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
	 * 2: 左ロータリーエンコーダのパルス波Aを入力
	 * 3: 左ロータリーエンコーダのパルス波Aを入力
	 * 4: LCD表示用(データバス)
	 * 5: LCD表示用(データバス)
	 * 6: LCD表示用(データバス)
	 * 7: LCD表示用(データバス)
	 *
	 */
	DDRC  = 0b11110011;
	PORTC = 0b00001100;
	
	/*
	 *	PORTD
	 *
	 * 0:
	 * 1:
	 * 2: 右ロータリーエンコーダのパルス波Aを入力
	 * 3: 右ロータリーエンコーダのパルス波Bを入力
	 * 4:
	 * 5:
	 * 6: 左モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 * 7: 左モーター用PWM出力(PWM出力にするときは必ずDDRを1にすること)
	 *
	 */
	DDRD  = 0b11000000;
	PORTD = 0b00001100;			//RE12D(ロータリーエンコーダの名前)は
								//プルアップ不要らしいが念のためプルアップは有効に
	
	//LCD初期化
	lcd_init();
	
	//タイマレジスタ設定
	//0:右モーターPWM
	Init_Timer0();
	//2:左モーターPWM
	Init_Timer2();
	//1:センサ用
	//Init_Timer1();
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
	sbi(PORTB, PB0);
	_delay_ms(1500);
	cbi(PORTB, PB0);
}

void beep_start(void)
{
	sbi(PORTB, PB0);
}

void beep_end(void)
{
	cbi(PORTB, PB0);
}

void encoder(void)
{
	static const int dir_right[] = {
		0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
	};
	static const int dir_left[] = {
		0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
	};
	
	static unsigned char left = 0, right = 0;
	
	right = (right << 2) + ((PIND >> 2) & 0b00000011); // PORTDの#3と#2
	left  = (left << 2)  + ((PINC >> 2) & 0b00000011); // PORTCの#3と#2
	
	Left_RotaryEncorder_val  += dir_left[left & 0b00001111];
	Right_RotaryEncorder_val += dir_right[right & 0b00001111];
}

//各センサの値とロータリーエンコーダのカウント数を同時にLCDに表示
void print_all_sensor(void)
{	
	sensor_distance_LF = (int)sensor_distance_convert_LF(LeftFront_Sensor_val);
	sensor_distance_RF = (int)sensor_distance_convert_RF(RightFront_Sensor_val);
	sensor_distance_L  = (int)sensor_distance_convert_L(Left_Sensor_val);
	sensor_distance_R  = (int)sensor_distance_convert_R(Right_Sensor_val);
	
	lcd_pos(0,0);
	lcd_number(Right_RotaryEncorder_val, 5);
	
	lcd_pos(0,6);
	lcd_number(Left_RotaryEncorder_val, 5);

	lcd_pos(1,0);
	lcd_number(sensor_distance_LF, 3);
	
	lcd_pos(1,4);
	lcd_number(sensor_distance_RF, 3);
	
	lcd_pos(1,8);
	lcd_number(sensor_distance_L, 3);
	
	lcd_pos(1,12);
	lcd_number(sensor_distance_R, 3);
	
	lcd_pos(0,0);
}

//各センサの値をLCDに表示
void Print_ADC(void)
{
	sensor_distance_LF = (int)sensor_distance_convert_LF(LeftFront_Sensor_val);
	sensor_distance_RF = (int)sensor_distance_convert_RF(RightFront_Sensor_val);
	sensor_distance_L  = (int)sensor_distance_convert_L(Left_Sensor_val);
	sensor_distance_R  = (int)sensor_distance_convert_R(Right_Sensor_val);
	
	lcd_str("RF  LF  L   R");
	
	lcd_pos(1,0);
	lcd_number(sensor_distance_RF, 3);
	
	lcd_pos(1,4);
	lcd_number(sensor_distance_LF, 3);
	
	lcd_pos(1,8);
	lcd_number(sensor_distance_L, 3);
	
	lcd_pos(1,12);
	lcd_number(sensor_distance_R, 3);
	
	lcd_pos(0,0);
}

void print_RotaryEncorder(void)
{
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
 *	Function Name	: Init_Timer1
 *	Tittle			: タイマー1のレジスタ設定
 *	Input			:	なし
 *	Output			: なし
 *	Descripution	: タイマーのオーバーフローにより割り込みを発生させる
 *	ISR(TIMER1_OVF_vect)
 */
void Init_Timer1(void)
{
	//TCCR1A(Timer Counter1 Control Register A)
	//	7,6: OC1Aから出力するPWM波の設定
	//		COM1A1=0, COM1A0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		#7 = 0, #6 = 0
	//
	//	5,4: OC1Bから出力するPWM波の設定
	//		COM1B1=0, COM1B0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		#5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//		#3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR1Bにも設定が跨っているので注意)
	//		WGM13=0, WGM12=0, WGM11=0, WGM10=0で通常動作(データシート p.84, 表16-5の番号4)
	//		#1 = 0, #0 = 0
	//		TODO 設定が間違っているのに何故か動く
	TCCR1A = 0b00000000;
	
	//TCCR1B(Timer Counter1 Control register B)
	//	7,6: ICNC1, ICES1 捕獲機道入力という謎の機能 (データシート p.82)
	//		今回は使用しない
	//		#7 = 0, #6 = 0
	//
	//	5: リザーブビット
	//		#5 = 0
	//
	//	4,3: PWM波形の種類の設定(上記に述べた設定の残り)
	//		#4 = 0, #3 = 1
	//
	//	2,1,0: 分周器設定
	//		ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//		分周は1/1024
	//		20MHz/8 ==> 約2500kHz(0.4us)
	//		#2 = 1, #1 = 0, #0 = 1
	//
	//		CTCがうまくいかなかったため
	//		オーバーフロー割り込みに変更した。
	//		ということなので
	//		0.4*2^16 = 0.0262s(38Hz)
	//		1秒間に38回サンプリングするがADCの仕組みから/4することになるので1個あたり1秒間に約10回になる
	TCCR1B = 0b00000010;
	
	//TCNT1(Timer Counter1)
	//		タイマカウンタ(16bit)に直接アクセスできる
	//		初期値をいれる
	//
	TCNT1 = 0;
	
	//OCR1A(Timer Counter1 Output Compare A Register)
	//      いつコンペアマッチAをさせるかを設定する(16bit)
	//
	OCR1A = 0;
	
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
	//		使用しないので
	//		#1 = 0
	//
	//	0 : 漏れ割り込み許可
	//		使用するので
	//		#0 = 1
	TIMSK1 = 0b00000001;
}

void Init_Timer3(void)
{
	//TCCR3A(Timer Counter3 Control Register A)
	//	7,6: OC3Aから出力するPWM波の設定
	//		COM3A1=0, COM3A0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		#7 = 0, #6 = 0
	//
	//	5,4: OC3Bから出力するPWM波の設定
	//		COM3B1=0, COM3B0=0で標準ポート動作 (データシート p.83, 表16-2)
	//		#5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//		#3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR3Bにも設定が跨っているので注意)
	//		WGM33=0, WGM32=1, WGM31=0, WGM30=0 CTCモード(データシート p.84, 表16-5の番号4)
	//		#1 = 0, #0 = 0
	TCCR3A = 0b00000000;
	
	//TCCR3B(Timer Counter3 Control register B)
	//	7,6: ICNC3, ICES3 捕獲起動入力という謎の機能 (データシート p.82)
	//		今回は使用しない
	//		#7 = 0, #6 = 0
	//
	//	5: リザーブビット
	//		#4 = 0
	//
	//	4,3: PWM波形の種類の設定(上記に述べた設定の残り)
	//		#4 = 0, #3 = 1
	//
	//	2,1,0: 分周器設定 (データシートp.85, 表16-6)
	//		ATmega1284P-AUの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//		メインクロックを64分周して使用する
	//		20MHz(0.05us)/ 64 = 312.5kHz(3.2us)
	//		#2 = 0, #1 = 1, #0 = 1
	TCCR3B = 0b00001011;
	
	//TCNT3(Timer Counter3)
	//		タイマカウンタ(16bit)に直接アクセスできる
	//		初期値をいれる
	TCNT3 = 0;
	
	//OCR3A(Timer Counter3 Output Compare A Register)
	//		いつコンペアマッチAをさせるかを設定する(16bit)
	//
	//		コンペアマッチAではロータリーエンコーダカウントと姿勢制御の割り込みを行う
	//		ロータリーエンコーダの回転を読むので、カウントレートがサンプリング周波数よりも、
	//		大きくなってはいけないので今回のサンプリング周波数は10kHz(100us)とする
	//		1クロックが312.5KHz(3.2us)なので
	//		100u / 3.2u = 31.25 今回は31にする
	//
	//		また同時に距離センサの割り込みを行う
	//		データシートのAD変換のところを見ると、
	//		変換時間は13-260us(50k-1MHz)と書いてある。
	//		今回AD変換の動作クロックは156kHzなので線形に推移すると仮定すると約240usになる。
	//
	//		AD変換時間は約240us またマルチプレクサの切り替え時間に50usいれている。
	//		一回のADCにかかる時間は250usになる。
	//
	//		AD変換が完了する前に割り込んでも意味がないので、割り込み間隔は250us以上にする必要がある。
	//		だいたい40Hz(0.025s)位(1個当たり1秒間に10サンプリング)すればいいと思うので
	//		コンペアマッチAの割り込み関数の中で250カウントする
	//		サンプリング周期は100usなので
	//		0.025 / (100 * 10^-6) = 250 より割り込み関数内で250カウントとることにする
	OCR3A = 31;
	
	//OCR3B(Timer Counter3 Output Compare B Register)
	//		いつコンペアマッチBをさせるかを設定する(16bit)
	//		だいたい40Hz(0.025s)位(1個当たり1秒間に10サンプリング)すればいいと思うので
	//		0.025s / (3.2 * 10^-6) = 7812.5なので、今回は7812とする。
	OCR3B = 0;

	//TIMSK3(Timer Counter 3 Interrupt Mask Register)
	//		タイマ割り込みを許可するためのレジスタ
	//
	//	7-3: リザーブビット
	//		 #7-3 = 0
	//
	//  2  : B比較の許可
	//		 使用しないので
	//		 #2 = 0
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
// vim: noet ts=4 sw=4 sts=0
