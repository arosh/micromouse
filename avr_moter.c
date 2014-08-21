/*
 * avr_moter.c
 *
 * Created: 2014/04/18 20:20:03
 *  Author: UEKI
 */ 

/*
 * specification
 *
 * 今回のマウスで使用するMDは http://www.maizuru-ct.ac.jp/control/machida/elec/motor/index.htm のTLP250を使用したHブリッジ回路である。
 *
 * 回路から分かるように内部LEDを吸い込み式で駆動させているためdutyが通常と逆になる。(例: duty比100%でモーター停止)
 * 普段と同様に使いたいため今回は波形出力のレジスタを変更し、コンペアマッチ時にlowを出力する半転出力動作を使用した。
 * 詳しくはデータシートを参照。もしかしたら解説をテキストとして残しておくかも。
 *
 * またサイトの駆動方法から分かるようにPWM出力モードとGPIOを切り替えて使用しているためそのあたりを留意すること。
 *
 */

#include <avr/io.h>
#define F_CPU 2000000
#include <util/delay.h>
#include "avr_moter.h"


//とりあえずの回転テスト用関数
void Init_CW_left(unsigned char pwm)
{
	TCCR0A = 0b10100011;		//出力設定初期化(AもBもPWM出力)
	TCCR0A = 0b11000011;		//OC0Bを通常ポートに変更
	PORTB  = 0b00010110;		//PB4をhighに
	OCR0A  = pwm;
}

//とりあえずの回転テスト用関数
void Init_CCW_left(unsigned char pwm)
{
	TCCR0A = 0b11110011;		//出力設定初期化(AもBもPWM出力)
	TCCR0A = 0b00110011;		//OC0Aを通常ポートに変更
	PORTB  = 0b00001110;		//PB3をhighに
	OCR0B  = pwm;
}


//とりあえずの回転テスト用関数
void Init_CW_right(unsigned char pwm)
{
	TCCR2A = 0b11110011;		//出力設定初期化(AもBもPWM出力)
	TCCR2A = 0b00110011;		//OC0Aを通常ポートに変更
	PORTD  = 0b10001111;		//PD7をhighに
	OCR2B  = pwm;
}

//とりあえずの回転テスト用関数
void Init_CCW_right(unsigned char pwm)
{
	TCCR2A = 0b11110011;		//出力設定初期化(AもBもPWM出力)
	TCCR2A = 0b11000011;		//OC0Bを通常ポートに変更
	PORTD  = 0b01001111;		//PD6をhighに
	OCR2A  = pwm;
}

/*
 *	Function Name : Init_Timer0
 *	Tittle        : タイマー0のレジスタ設定
 *	Input		  :	なし
 *	output        :	OCR0A, OCR0B
 *	Descripution  : 高速PWMモード使用しPWM波を出力
 *					
 */
 
void Init_Timer0(void)
{
	/*
	 * TCCR0A(Timer Counter0 Control Register A)
	 *
	 *	7,6: OC0Aから出力するPWM波の設定
	 *       コンペアマッチでHIGH、BOTTOMでLOWを出力
	 *		 #7 = 1, #6 = 1
	 *
	 *	5,4: OC0Bから出力するPWM波の設定
	 *       コンペアマッチでHIGH、BOTTOMでLOWを出力
	 *       #5 = 1, #4 = 1
	 *
	 *	3,2: リザーブビット
	 *       #3 = 0, #2 = 0
	 *
	 *	1,0: PWM波形の種類の設定(下記のTCCR0Bにも設定が跨っているので注意)
	 *		 8bit高速PWM動作モード
	 *       #1 = 1, #0 = 1
	 */
	TCCR0A = 0b11110011;
	
	/*
	 * TCCR0B(Timer Counter0 Control register B)
	 *
	 *	7,6: OC0A,OC0B 強制変更設定
	 *		 これは非PWMモードを使用する際に設定する　今回は使用しない
	 *		 #7 = 0, #6 = 0
	 *
	 *	5,4: リザーブビット
	 *		 #5 = 0, #4 = 0
	 *
	 *	3  : PWM波形の種類の設定(上記に述べた設定の残り)
	 *       #3 = 0
	 *
	 *	2,1,0: 分周器設定
	 *			ATmaga88Pの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	 *			MNCTのサイトでは実用で48kHzがいいとあるが、分周の関係で10kか78kになってしまう。
	 *			78kだと電流がながれにくくなりFETが発熱するので、騒音は出るが10kで動かすことにする。
	 *			20MHz/0xFF(255)/8 ==> 約10kHz　計算式はDatasheetを参照
	 *			#2 = 0, #1 = 1, #0 = 0
	 */
	TCCR0B = 0b00000010;
	
	/*
	 * TCNT0(Timer Counter0)
	 *
	 *		タイマカウンタ(8bit)に直接アクセスできる
	 *		初期値をいれる(今回は使用しない)
	 */
	TCNT0 = 0b00000000;
	
	/*
	 * OCR0A(Timer Counter0 Output Compare A Register)
	 *
	 *		コンペアマッチAのタイミング設定(8bit)
	 *		default duty 0%
	 */
	OCR0A = 0b00000000;
	
	/*
	 * OCR0B(Timer Counter0 Output Compare B Register)
	 *		
	 *		コンペアマッチBのタイミング設定(8bit)
	 *		default dety 0%
	 */
	OCR0B = 0b00000000;
}

/*
 *	Function Name : Init_Timer2
 *	Tittle        : タイマー2のレジスタ設定
 *	Input		  :	なし
 *	output        :	OCR2A, OCR2B
 *	Descripution  : 高速PWMモードを使用しPWM波を出力する
 *
 */
 
void Init_Timer2(void)
{
	/*
	 * TCCR2A(Timer Counter2 Control Register A)
	 *
	 *	7,6: OC0Aから出力するPWM波の設定
	 *       コンペアマッチでHIGH、BOTTOMでLOWを出力
	 *		 #7 = 1, #6 = 1
	 *
	 *	5,4: OC0Bから出力するPWM波の設定
	 *       コンペアマッチでHIGH、BOTTOMでLOWを出力
	 *       #5 = 1, #4 = 1
	 *
	 *	3,2: リザーブビット
	 *       #3 = 0, #2 = 0
	 *
	 *	1,0: PWM波形の種類の設定(下記のTCCR0Bにも設定が跨っているので注意)
	 *		 8bit高速PWM動作モード
	 *       #1 = 1, #0 = 1
	 */
	TCCR2A = 0b11110011;
	
	/*
	 * TCCR2B(Timer Counter2 Control register B)
	 *
	 *	7,6: OC0A,OC0B 強制変更設定
	 *		 これは非PWMモードを使用する際に設定する　今回は使用しない
	 *		 #7 = 0, #6 = 0
	 *
	 *	5,4: リザーブビット
	 *		 #5 = 0, #4 = 0
	 *
	 *	3  : PWM波形の種類の設定(上記に述べた設定の残り)
	 *       #3 = 0
	 *
	 *	2,1,0: 分周器設定
	 *         ATmaga88Pの動作クロックは20MHz(ヒューズビットで分周設定を解除後)
	 *         MNCTのサイトでは実用で48kHzがいいとあるが、分周の関係で10kか78kになってしまう。
	 *		   78kだと電流がながれにくくなりFETが発熱するので、騒音は出るが10kで動かすことにする。
	 *         20MHz/0xFF(255)/8 ==> 約10kHz　計算式はDatasheetを参照
	 *         #2 = 0, #1 = 1, #0 = 0
	 */
	TCCR2B = 0b00000010;
	
	/*
	 * TCNT2(Timer Counter2)
	 *
	 *		タイマカウンタ(8bit)に直接アクセスできる
	 *		初期値をいれる(今回は使用しない)
	 */
	TCNT2 = 0b00000000;
	
	/*
	 * OCR2A(Timer Counter2 Output Compare A Register)
	 *
	 *      いつコンペアマッチAをさせるかを設定する(8bit)
	 *		default duty 0%
	 */
	OCR2A = 0b00000000;
	
	/*
	 * OCR2B(Timer Counter2 Output Compare B Register)
	 *		
	 *		いつコンペアマッチBをさせるかを設定する(8bit)
	 *		default dety 0%
	 */
	OCR2B = 0b00000000;
}
