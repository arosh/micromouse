/*
 * micromouse_ver0.c
 *
 * Created: 2014/04/07 21:16:13
 *  Author: takemichi
 */ 


#include <avr/io.h>
#define F_CPU 20000000
#include <util/delay.h>
#include <avr/interrupt.h>
#include "avr_lcd.h"

void Inti_Timer(void);					//タイマの初期設定
void Init_ADC(void);					//AD変換の初期化設定
void Inti_ADC_get(void);				//AD変換のchを切り替える関数
void digit_partition(void);				//LCD表示のために取得値を桁ごとに分割

//void encoder(void);

//各センサ値を格納する関数
volatile unsigned char Left_cnt;
volatile unsigned char LeftFront_cnt;
volatile unsigned char RightFront_cnt;
volatile unsigned char Right_cnt;

struct{
	
	volatile unsigned char dig100;
	volatile unsigned char dig10;
	volatile unsigned char dig1;
	
}Left = {0, 0, 0}, LeftFront = {0, 0, 0}, RightFront ={0, 0, 0}, Right ={0, 0, 0};

ISR(TIMER1_COMPA_vect){
		
		Inti_ADC_get();
}

int main(void)
{		
	cli();
	
	DDRA  = 0xF0;
	PORTA = 0x00;
	DDRC  = 0xFF;					
	PORTC = 0x00;
	DDRD  = 0xFF;						//PORTDを出力に設定
	PORTD = 0x00;						
	DDRB  = 0xFF;						//PORTBを出力に設定						
	PORTB = 0x00;						//コンデンサチャージ開始
	
	lcd_init();							//LCD初期化
	Inti_Timer();						//タイマーレジスタ設定
	Init_ADC();							//AD変換レジスタ設定
	
	sei();
	
	while(1){
		
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

	return 0;
}

/*
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
*/

//センサ値の桁をわける(LCDの文字列表示のため) 
void digit_partition(void)
{	
	//前(左側の)
	Left.dig1    =  Left_cnt % 10;
	Left.dig10   = (Left_cnt / 10) % 10;
	Left.dig100  = (Left_cnt / 100) % 10; 

	//左のセンサ
	LeftFront.dig1    =  LeftFront_cnt % 10;
	LeftFront.dig10   = (LeftFront_cnt / 10) % 10;
	LeftFront.dig100  = (LeftFront_cnt / 100) % 10;

	//右のセンサ
	RightFront.dig1    =  RightFront_cnt % 10;
	RightFront.dig10   = (RightFront_cnt / 10) % 10;
	RightFront.dig100  = (RightFront_cnt / 100) % 10;

	//前(右側)
	Right.dig1    =  Right_cnt % 10;
	Right.dig10   = (Right_cnt / 10) % 10;
	Right.dig100  = (Right_cnt / 100) % 10;

}

void Inti_ADC_get(void)
{
	
//--------------------------------------------------------------------------------------------------------
	//AD_ch0

	PORTB = 0b00000001;			//LED(ch0)発行
	
	ADMUX = 0b00100000;			//入力をch0に切り替え
	_delay_ms(5);				//切り替えが安定するまで待機

	ADCSRA = 0b11000111;		//AD変換スタート		#6 = 1 にすると変換がスタートする
	while(ADCSRA & 0b01000000);	// 変換が終了するまで待つ	変換結果が終わると自動的に#6 = 0になる
	
	Left_cnt = ADCH;			//値を確保

	PORTB = 0b00000000;			//リチャージ

	//--------------------------------------------------------------------------------------------------------
	//AD_ch1
	
	PORTB = 0b00000010;			//LED(ch1)発光
	
	ADMUX = 0b00100001;			//入力をch1に切り替え
	_delay_us(5);				//切り替えが安定するまで待機
	
	ADCSRA = 0b11000111;		//AD変換スタート		#6 = 1 にすると変換がスタートする
	while(ADCSRA & 0b01000000);	//変換が終了するまで待つ	変換結果が終わると自動的に#6 = 0になる
	
	LeftFront_cnt = ADCH;		//値を確保

	PORTB = 0b00000000;			//リチャージ
	
	//--------------------------------------------------------------------------------------------------------
	//AD_ch2
	
	PORTB = 0b00000100;			//LED(ch2)発光
	
	ADMUX = 0b00100010;			//入力をch2に切り替え
	_delay_us(5);				//切り替えが安定するまで待機

	ADCSRA = 0b11000111;		//AD変換スタート		#6 = 1 にすると変換がスタートする
	while(ADCSRA & 0b01000000);	//変換が終了するまで待つ	変換結果が終わると自動的に#6 = 0になる
	
	RightFront_cnt = ADCH;		//値を確保

	PORTB = 0b00000000;			//リチャージ
	
	//-------------------------------------------------------------------------------------------------------
	//AD_ch3
	
	PORTB = 0b00001000;			//LED(ch3)発光
	
	ADMUX = 0b00100011;			//入力をch3に切り替え
	_delay_us(5);				//切り替えが安定するまで待機
	
	ADCSRA = 0b11000111;		//AD変換スタート		#6 = 1 にすると変換がスタートする
	while(ADCSRA & 0b01000000);	// 変換が終了するまで待つ	変換結果が終わると自動的に#6 = 0になる

	Right_cnt = ADCH;			//値を確保

	PORTB = 0b00000000;			//リチャージ

}


//***********************************************************************************//
//	Function Name :	Init_ADC														*//
//	Titlle        : AD変換用レジスタの設定											*//
//  Input         :	ADC0, ADC1, ADC2, ADC3, ADC4, ADC5								*//
//  output        :																	*//
//	Description   :	通常動作モード													*//
//***********************************************************************************//
void Init_ADC(void)
{
	//ADMUX(ADC Multiplexer Selct Register)
	//	7: リザーブビット
	//		#7 = 0
	
	//	6: 基準電圧選択
	//		基準電圧としてVcc(5V)を使用する
	//		#6 = 0
	
	//	5: 変換結果を右寄せにするか左寄せにするかを設定する
	//		左寄せにする
	//		#5 = 1
	
	//	4: リザーブビット
	//		#4 = 0
	
	//	3,2,1,0: ADチャンネル選択
	//		このビットをAD変換中にしても変換完了までは実行されない
	//		とりあえずADC0に設定
	//		#3 = 0, #2 = 0, #1 = 0, #0 = 0
	ADMUX = 0b00100000;
	
	//ADCSRA(ADC Control and Status Register A)
	//	7: AD変換許可
	//		AD変換を許可する
	//		#7 = 1
	
	//	6: ADSC(ADC Start Conversion)	AD変換開始
	//		とりあえず開始はまだしない
	//		#6 = 0
	
	//	5: AD変換自動起動許可
	//		#5 = 0
	
	//	4: AD変換完了割り込み要求フラグ
	//		AD変換が完了し結果のレジスタが更新されるとこのフラグが'1'になる
	//		とりあえず初期値を入力しておく
	//		#4 = 0
	
	//	3: AD変換完了割り込み許可
	//		割り込みを使用する場合は'1'にしておこう
	//		#3 = 0
	
	//	2,1,0: AD変換クロック選択
	//		ADCは変換スピードを早くしすぎると10ビット分しっかり
	//		機能しないので50kHz～200KHzのクロック周波数に設定する
	//		ATmega88Pの動作クロックは20MHzなので、
	//		20M/128 ==> 156kHzとする　分周は/φ128
	//		#2 = 1, #1 = 1, #0 = 1
	ADCSRA = 0b10000110;
	
	//ADCSRB(ADC Control and Status Register B)
	//	7:	リザーブビット
	//		#7 = 0
	
	//	6:	よくわからん
	//		#6 = 0
	
	//	5,4,3:	リザーブビット
	//		#5 = 0, #4 = 0, #3 = 0
	
	//	2,1,0:	AD変換自動起動要因選択
	//		連続変換動作
	//		#2 = 0, #1 = 0, #0 = 0
	ADCSRB = 0b00000000;
	
	//DIDR0(Digital Input Disable Register 0)
	// 7,6: リザーブビット
	//		#7 = 0, #6 = 0
	
	// 5,4,3,2,1,0: デジタル入力禁止
	//		今回はAD変換のみで使用するので
	//		すべて1にする
	DIDR0 = 0b00001111;
}


//***************************************************************************//
//*	Function Name : レジスタ初期化関数										*//
//*	Tittle        : タイマー0のレジスタ設定									*//
//*	Input		  :	なし													*//
//*	output        :	なし												　　*//
//*	Descripution  : CTCを使って手軽にカウントする							*//
//*					ISR(TIMER0_COMPA_vect)									*//
//***************************************************************************//
void Inti_Timer(void)
{
	//TCCR0A(Timer Counter0 Control Register A)
	//	7,6: OC0Aから出力するPWM波の設定
	//       何も出力しないので
	//		 #7 = 0, #6 = 0
	//
	//	5,4: OC0Bから出力するPWM波の設定
	//       何も出力しないので
	//       #5 = 0, #4 = 0
	//
	//	3,2: リザーブビット
	//       #3 = 0, #2 = 0
	//
	//	1,0: PWM波形の種類の設定(下記のTCCR0Bにも設定が跨っているので注意)
	//		 CTCモード
	//       #1 = 1, #0 = 0
	TCCR1A = 0b00000010;
	
	//TCCR0B(Timer Counter0 Control register B)
	//	7,6: OC0A,OC0B 強制変更設定
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
	//         ATmega88Pの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	//         分周は1/1024
	//         20MHz/1024 ==> 約19kHz
	//         #2 = 1, #1 = 0, #0 = 1
	TCCR1B = 0b0000101;
	
	//TCNT0(Timer Counter0)
	//		タイマカウンタ(8bit)に直接アクセスできる
	//		初期値をいれる
	TCNT1 = 0;
	
	
	//OCR0A(Timer Counter0 Output Compare A Register)
	//      いつコンペアマッチAをさせるかを設定する(8bit)
	//		今回は1秒カウントしたい
	//      1クロックは19kHz(5*10^-5) 今回は100msをつくる。
	//		100m/5*10^-5 = 約
	OCR1A = 5;
	
	//OCR0B(Timer Counter0 Output Compare B Register)
	//		いつコンペアマッチBをさせるかを設定する(8bit)
	//		今回は使用しない。
	OCR1B = 0;

	//TIMSK0(Timer Counter 0 Interrupt Mask Register)
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
