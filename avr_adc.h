/*
 * avr_adc.h
 *
 * Created: 2014/04/22 14:38:09
 *  Author: takemichi
 */


#ifndef AVR_ADC_H_
#define AVR_ADC_H_

//ACDのレジスタ設定
void Init_ADC(void);

//AD変換のchを切り替えて値を取得していく関数
void Init_ADC_get(void);

#endif /* AVR_ADC_H_ */
