/*
 * avr_moter.h
 *
 * Created: 2014/04/18 20:21:16
 *  Author: UEKI
 */ 


#ifndef AVR_MOTER_H_
#define AVR_MOTER_H_

//タイマー0のレジスタ設定(8bit高速PWMモード)
void Inti_Timer0(void);

//タイマー2のレジスタ設定(8bit高速PWMモード)
void Inti_Timer2(void);

//とりあえずの回転テスト用関数
void Inti_CW_right(unsigned char pwm);

//とりあえずの回転テスト用関数
void Inti_CCW_right(unsigned char pwm);

void Inti_CW_left(unsigned char pwm);

void Inti_CCW_left(unsigned char pwm);

#endif /* AVR_MOTER_H_ */