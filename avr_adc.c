/*
 * avr_adc.c
 *
 * Created: 2014/04/22 14:37:38
 *  Author: takemichi
 */
#include <avr/io.h>
#define F_CPU 20000000
#include <avr/interrupt.h>
#include <util/delay.h>
#include "avr_adc.h"

volatile unsigned char Left_Sensor_val = 0;
volatile unsigned char LeftFront_Sensor_val = 0;
volatile unsigned char RightFront_Sensor_val = 0;
volatile unsigned char Right_Sensor_val = 0;
volatile unsigned char adc_chanel = 0;

ISR(ADC_vect){
	switch(adc_chanel){
		case 0:
      RightFront_Sensor_val = ADCH;
      break;

		case 1:
      LeftFront_Sensor_val = ADCH;
      break;
		
		case 2:
      Left_Sensor_val = ADCH;
      break;
		
		case 3:
      Right_Sensor_val= ADCH;
      break;
	}
	
	PORTA = 0b00000000;			//LED発光停止
	
	adc_chanel++;
	
	if(adc_chanel == 4){
		adc_chanel = 0;
	}
}

void Init_ADC_get(void)
{
  //                                RightFront  LeftFront      Left       Right
	const unsigned char LEDPORT[] = { 0b10000000, 0b00010000, 0b00100000, 0b01000000 };
  // Init_ADCの記述と整合性がとれていなかったので、0b0010... から 0b0110...に変更
  // 現在順番がおかしい
	const unsigned char MUXREG[]  = { 0b01100000, 0b01100001, 0b01100010, 0b01100011 };

	PORTA = LEDPORT[adc_chanel];			//LED(ch0)発光
	
	ADMUX = MUXREG[adc_chanel];			//入力をch0に切り替え
	_delay_us(50);						//切り替えが安定するまで待機
  ADCSRA |= (1 << 6);				//AD変換スタート		#6 = 1 にすると変換がスタートする
}

/*
 *
 *	Function Name :	Init_ADC
 *	Title         : AD変換用レジスタの設定
 *  Input         :	ADC0, ADC1, ADC2, ADC3, ADC4, ADC5								
 *  Output        :	なし
 *	Description   :	通常動作モード
 *
 */
void Init_ADC(void)
{
	/*
	 * ADMUX(ADC Multiplexer Selct Register, データシート p.162)
	 *	7,6: 基準電圧選択
	 *		基準電圧としてAVCC(5V)を使用する
	 *		#7 = 0, #6 = 1
	 *
	 *	5: 変換結果を右寄せにするか左寄せにするかを設定する
	 *		左寄せにする
	 *		#5 = 1
	 *
	 *	4,3,2,1,0: ADチャンネル選択
	 *		このビットをAD変換中にしても変換完了までは実行されない
	 *		とりあえずADC0に設定
	 *		#4 = 0, #3 = 0, #2 = 0, #1 = 0, #0 = 0
	 */
	ADMUX = 0b01100000;
	
	/*
	 * ADCSRA(ADC Control and Status Register A, データシート p.163)
	 *	
	 *	7: AD変換許可
	 *		AD変換を許可する
	 *		#7 = 1
	 *
	 *	6: ADSC(ADC Start Conversion)	AD変換開始
	 *		とりあえず開始はまだしない
	 *		#6 = 0
	 *
	 *	5: AD変換自動起動許可
	 *		#5 = 0
	 *
	 *	4: AD変換完了割り込み要求フラグ
	 *		AD変換が完了し結果のレジスタが更新されるとこのフラグが'1'になる
	 *		とりあえず初期値を入力しておく
	 *		#4 = 1
	 *
	 *	3: AD変換完了割り込み許可
	 *		割り込みを使用する場合は'1'にしておく
	 *		#3 = 1
	 *
	 *	2,1,0: AD変換クロック選択
	 *		ADCは変換スピードを早くしすぎると10ビット分しっかり
	 *		機能しないので50kHz〜200KHzのクロック周波数に設定する
	 *		ATmega88Pの動作クロックは20MHzなので、
	 *		20M/128 ==> 156kHzとする　分周は/φ128
	 * 		#2 = 1, #1 = 1, #0 = 1
	 */
	ADCSRA = 0b10000111;
	
	/*
	 * ADCSRB(ADC Control and Status Register B)
	 *	7:	リザーブビット
	 *		#7 = 0
	 *
	 *	6:	よくわからん
	 *		#6 = 0
	 *
	 *	5,4,3:	リザーブビット
	 *		#5 = 0, #4 = 0, #3 = 0
	 *
	 *	2,1,0:	AD変換自動起動要因選択 (データシートp.164, 表24-6)
	 *		連続変換動作
	 *		#2 = 0, #1 = 0, #0 = 0
	 */
	ADCSRB = 0b00000000;
	
	/*
	 * DIDR0(Digital Input Disable Register 0)
	 *
   * デジタル入力を使用しないポートは消費電力削減のために1を書いて入力を禁止する
	 *	
	 */
	DIDR0 = 0b00001111;
}
// vim: noet ts=4 sw=4 sts=0
