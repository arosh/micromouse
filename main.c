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
#include <stdlib.h>
#include <stdbool.h>
#include "sensor.h"
#include "agent.h"

void serial_number(long int value, int digit);

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

void rs_putc (char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0); //UDREビットが1になるまで待つ
	UDR0 = c;
}

void rs_puts (char *st)
{
	while (*st) {
		rs_putc (*st);
		if (*(st++) == '\n') rs_putc('\r');
	}
}

//AD変換値を距離[mm]に変換(RF)
float sensor_distance_convert_RF(int x)
{
	return 0.0052 * x * x - 1.597 * x + 131.8;
}

//AD変換値を距離[mm]に変換(LF)
float sensor_distance_convert_LF(int x)
{
	return 0.0045 * x * x - 1.5011 * x + 132.6;
}

//AD変換値を距離[mm]に変換(L)
float sensor_distance_convert_L(int x)
{
	return 0.0025 * x * x - 0.9635 * x + 132.45;
}

//AD変換値を距離[mm]に変換(R)
float sensor_distance_convert_R(int x)
{
	return 0.0023 * x * x - 0.9322 * x + 133.16;
}

//パルス速度をPWMに変換(右)
int pulsevelocity_PWM_convert_R(int x)
{
	return 0.0228 * x + 12.783;
}

//パルス速度をPWMに変換(左)
int pulsevelocity_PWM_convert_L(int x)
{
	return 0.025 * x + 11.967;
}


volatile float sensor_distance_LF;
volatile float sensor_distance_RF;
volatile float sensor_distance_L;
volatile float sensor_distance_R;
volatile float sensor_distance_AVE_LF_RF;

uint8_t sensor_get(void){
	
	bool wall_r = sensor_distance_R < 90;
	bool wall_l = sensor_distance_L < 90;
	bool wall_f = sensor_distance_AVE_LF_RF < 40;
	
	uint8_t x = 0;
	
	if(wall_f == 1){
		sbi(x, 0);
	}
	if(wall_l == 1){
		sbi(x, 1);
	}
	if(wall_r == 1){
		sbi(x, 2);
	}
	
	return x;
}

volatile unsigned loop_count;

//ロータリーエンコーダの値を格納する変数
volatile long int Left_RotaryEncorder_val  = 0;
volatile long int Right_RotaryEncorder_val = 0;

//ホイールのパルス速度(負の場合もあるのでsignedで)
volatile int pulse_velocity_left;
volatile int pulse_velocity_right;

//速度平均を求めるための変数
volatile int time = 0;
volatile int spd_R[3] = {0, 0, 0};
volatile int spd_L[3] = {0, 0, 0};
volatile int ave_spd_R = 0;
volatile int ave_spd_L = 0;

volatile long int movement_right = 0;
volatile long int movement_left  = 0;

//ターンに必要なフラグ達
volatile char turn_flag = 0;
volatile char prefer_turn_flag = 0;
volatile long int prefer_turn_right = 0;
volatile long int prefer_turn_left  = 0;

volatile int error_turn_right;
volatile int error_turn_left;

volatile int turn_select = -1;	// 1なら右回転	-1なら左回転

volatile char hips_flag = 0;

volatile char mode = 8;
volatile int time_hips;
volatile char old_mode = 0;
volatile bool need_hips;
volatile int masu = 0;

void ave_speed(void)
{
	static long int old_pulse_R = 0;
	static long int old_pulse_L = 0;
	long int now_pulse_R = Right_RotaryEncorder_val;
	long int now_pulse_L = Left_RotaryEncorder_val;
	int i;
	
	//この割り込み関数は10msごとに読み込まれるのでパルスカウントの差を10ms[0.01s]で割ることでパルス速度[pulse/s]を求めることができる
	spd_R[time] = (now_pulse_R - old_pulse_R) * 100;
	spd_L[time] = (now_pulse_L - old_pulse_L) * 100;
	old_pulse_R = now_pulse_R;
	old_pulse_L = now_pulse_L;
	
	time++;
	if(time == 3){
		time = 0;
	}
	
	for(i = 0; i < 3;i++){
		ave_spd_R += spd_R[i];
		ave_spd_L += spd_L[i];
	}
	
	ave_spd_R /= 3;
	ave_spd_L /= 3;
	
}

void sensor_convert(void)
{
	sensor_distance_LF = sensor_distance_convert_LF(LeftFront_Sensor_val);
	sensor_distance_RF = sensor_distance_convert_RF(RightFront_Sensor_val);
	sensor_distance_L  = sensor_distance_convert_L(Left_Sensor_val);
	sensor_distance_R  = sensor_distance_convert_R(Right_Sensor_val);
	sensor_distance_AVE_LF_RF = (sensor_distance_LF+sensor_distance_RF)*0.5;
}

void forward_hips(void)
{
	//速度の誤差にかけるゲイン
	const float Kp_velocity_right = 0.07;
	const float Kp_velocity_left  = 0.085;
		
	//左右の移動量の誤差にかけるゲイン
	const float Kp_movement_right = 0.26;
	const float Kp_movement_left  = 0.26;
		
	//目標パルス速度
	const int preferrance_pluse_velocity_right = 1000;
	const int preferrance_pluse_velocity_left  = 1000;
		
	//目標パルス速度と現在のパルス速度の誤差
	float error_velocity_left;
	float error_velocity_right;
		
	//左右の移動量の誤差
	int error_movement_left;
	int error_movement_right;
		
	//目標パルス速度にするための制御量
	int control_velocity_right;
	int control_velocity_left;
		
	//左右の移動量の誤差に対する制御量
	int control_movement_right;
	int control_movement_left;
		
	//左右の速度の誤差
	error_velocity_left  = preferrance_pluse_velocity_left  - ave_spd_L;
	error_velocity_right = preferrance_pluse_velocity_right - ave_spd_R;
		
	//左右の移動量の誤差
	error_movement_left  = movement_right - movement_left;
	error_movement_right = movement_left - movement_right;
		
	//目標パルス速度にするための制御量
	control_velocity_right = (int)(Kp_velocity_right * error_velocity_right);
	control_velocity_left  = (int)(Kp_velocity_left * error_velocity_left);
		
	//左右の壁の誤差に対する制御量
	control_movement_right = (int)(Kp_movement_right * error_movement_right);
	control_movement_left  = (int)(Kp_movement_left  * error_movement_left);
		
	motor_right(control_velocity_right + control_movement_right);
	motor_left(control_velocity_left  + control_movement_left);
}



void forward(void)
{
	//速度の誤差にかけるゲイン
	const float Kp_velocity_right = 0.07;
	const float Kp_velocity_left  = 0.085;
	
	//左右の移動量の誤差にかけるゲイン
	const float Kp_movement_right = 0.26;
	const float Kp_movement_left  = 0.26;
	
	//目標パルス速度
	const int preferrance_pluse_velocity_right = 1600;
	const int preferrance_pluse_velocity_left  = 1600;
	
	//目標パルス速度と現在のパルス速度の誤差
	float error_velocity_left;
	float error_velocity_right;
	
	//左右の移動量の誤差
	int error_movement_left;
	int error_movement_right;
	
	//目標パルス速度にするための制御量
	int control_velocity_right;
	int control_velocity_left;
	
	//左右の移動量の誤差に対する制御量
	int control_movement_right;
	int control_movement_left;
	
	//左右の速度の誤差
	error_velocity_left  = preferrance_pluse_velocity_left  - ave_spd_L;
	error_velocity_right = preferrance_pluse_velocity_right - ave_spd_R;
	
	//左右の移動量の誤差
	error_movement_left  = movement_right - movement_left;
	error_movement_right = movement_left - movement_right;
	
	//目標パルス速度にするための制御量
	control_velocity_right = (int)(Kp_velocity_right * error_velocity_right);
	control_velocity_left  = (int)(Kp_velocity_left * error_velocity_left);
	
	//左右の壁の誤差に対する制御量
	control_movement_right = (int)(Kp_movement_right * error_movement_right);
	control_movement_left  = (int)(Kp_movement_left  * error_movement_left);
	
	motor_right(control_velocity_right + control_movement_right);
	motor_left(control_velocity_left  + control_movement_left);
	
}

void speed_down(void)
{	
	motor_brake_right();
	motor_brake_left();
}

void turn_right(void)
{
	volatile int turn_select = 1;	// 1なら右回転	-1なら左回転
	
	const float Kp_turn_right = 0.55;
	const float Kp_turn_left  = 0.60;
	const float Kd_turn_right = 0.15;
	const float Kd_turn_left  = 0.15;
	
	static int old_error_turn_right = 0;
	static int old_error_turn_left = 0;
	
	int P_control_turn_right;
	int P_control_turn_left;
	
	int D_control_turn_right;
	int D_control_turn_left;
	
	if(prefer_turn_flag == 1){
		
		prefer_turn_right = Right_RotaryEncorder_val + (-turn_select * 180);		//タイヤ間円の円周の1/4がパルスカウントの180と一致する
		prefer_turn_left  = Left_RotaryEncorder_val  + (turn_select  * 180);
		
		prefer_turn_flag = 0;
		
	}
	
	error_turn_right = prefer_turn_right - Right_RotaryEncorder_val;
	error_turn_left  = prefer_turn_left  - Left_RotaryEncorder_val;
	
	P_control_turn_right = (int)(Kp_turn_right * error_turn_right);
	P_control_turn_left  = (int)(Kp_turn_left  * error_turn_left);
	
	D_control_turn_right = (int)(Kd_turn_right * (error_turn_right - old_error_turn_right));
	D_control_turn_left  = (int)(Kd_turn_left  * (error_turn_left  - old_error_turn_left));
	
	motor_right(P_control_turn_right + D_control_turn_right);
	motor_left(P_control_turn_left + D_control_turn_left);
	
	//過去の偏差を更新
	old_error_turn_left  = error_turn_left;
	old_error_turn_right = error_turn_right;
	
}

void turn_left(void)
{
	volatile int turn_select = -1;	// 1なら右回転	-1なら左回転
	
	const float Kp_turn_right = 0.45;
	const float Kp_turn_left  = 0.50;
	const float Kd_turn_right = 0.15;
	const float Kd_turn_left  = 0.15;
	
	static int old_error_turn_right = 0;
	static int old_error_turn_left = 0;
	
	int P_control_turn_right;
	int P_control_turn_left;
	
	int D_control_turn_right;
	int D_control_turn_left;
	
	if(prefer_turn_flag == 1){
		
		prefer_turn_right = Right_RotaryEncorder_val + (-turn_select * 180);		//タイヤ間円の円周の1/4がパルスカウントの180と一致する
		prefer_turn_left  = Left_RotaryEncorder_val  + (turn_select  * 180);
		
		prefer_turn_flag = 0;
		
	}
	
	error_turn_right = prefer_turn_right - Right_RotaryEncorder_val;
	error_turn_left  = prefer_turn_left  - Left_RotaryEncorder_val;
	
	P_control_turn_right = (int)(Kp_turn_right * error_turn_right);
	P_control_turn_left  = (int)(Kp_turn_left  * error_turn_left);
	
	D_control_turn_right = (int)(Kd_turn_right * (error_turn_right - old_error_turn_right));
	D_control_turn_left  = (int)(Kd_turn_left  * (error_turn_left  - old_error_turn_left));
	
	motor_right(P_control_turn_right + D_control_turn_right);
	motor_left(P_control_turn_left + D_control_turn_left);
	
	//過去の偏差を更新
	old_error_turn_left  = error_turn_left;
	old_error_turn_right = error_turn_right;
	
}

void hips(void)
{
	motor_left(-45);
	motor_right(-45);
	time_hips++;
}


int mode_sel(void)
{
	bool wall_r = sensor_distance_R < 90;
	bool wall_l = sensor_distance_L < 90;
	bool wall_f = sensor_distance_AVE_LF_RF < 40;
	
	if(wall_l == false) {
		prefer_turn_flag = 1;
		need_hips = wall_r;
		return 3;
	}
	
	if(wall_f == false) {
		prefer_turn_flag = 0;
		need_hips = false;
		return 1;
	}
	
	if(wall_r == false) {
		prefer_turn_flag = 1;
		need_hips = wall_l;
		return 2;
	}
	
	//180turn
	prefer_turn_flag = 1;
	need_hips = true;
	return 4;
}

void lcd_check(void){
	sensor_convert();
	lcd_pos(0,0);
	lcd_str("mode");
	lcd_pos(0,5);
	lcd_number(mode, 1);
	lcd_pos(0,8);
	lcd_number(masu, 5);
	lcd_pos(1,0);
	lcd_number(movement_left, 5);
	lcd_pos(1,9);
	lcd_number(movement_right, 5);
}

// センサ用割り込み
ISR(TIMER1_COMPA_vect){
	ave_speed();		//速度の平均をとる
	
	switch(mode){
		
		case 1:
		movement_right += spd_R[time] * 0.01;	//速度を積分
		movement_left  += spd_L[time] * 0.01;
		
		forward();
		break;
		
		case 2:
		turn_right();
		break;
		
		case 3:
		turn_left();
		break;
		
		case 4:
		turn_right();
		break;
		
		case 5:
		hips();
		break;
		
		case 6:
		movement_right += spd_R[time] * 0.01;	//速度を積分
		movement_left  += spd_L[time] * 0.01;
		forward_hips();
		break;
		
		case 7:
		speed_down();
		break;
		
		case 8:
		//何もしないで止まる.
		motor_brake_left();
		motor_brake_right();
		break;
	}
}



// エンコーダ用割り込み
ISR(TIMER3_COMPA_vect)
{
	static int sensor_count = 0;
	
	if(sensor_count >= 250){
		Init_ADC_get();
		sensor_count = 0;
	}
	sensor_count++;
	
	encoder();

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
	* 0: シリアル通信(RXD)
	* 1: シリアル通信(TXD)
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
	Init_Timer1();
	//3:エンコーダ読み取り+姿勢制御
	Init_Timer3();
	
	//AIの初期化
	agent_init();
	
	//ADCの初期化
	Init_ADC();
	
	//UBRR0  = 129;
	//UCSR0A = 0b00000000;
	//UCSR0B = 0b00011000;
	//UCSR0C = 0b00000110;
	sei();
	
	loop_until_bit_is_clear(PINB,PINB2);		//スタートスイッチ(青色)が押されるまで待機
	
	beep_start();										//ブザーを鳴らす
	_delay_ms(1000);
	beep_end();
	
	while(1){
		
		//モードによって動作を切り替える
		//	1: 前進
		//	2: 右90度回転
		//	3: 左90度回転
		//	4: 袋小路(90度回転)
		//	5: ケツをつける
		//	6: ケツをつけた分、前に前進
		//	7: センサによるブレーキ
		
		
		sensor_convert();	//AD変換値を距離[mm]に変換
		
		mode = 1;
		time_hips = 0;
		movement_left  = 0;
		movement_right = 0;
		
		while(!(abs(movement_left) >= 10 && abs(movement_right) >= 10 && sensor_distance_AVE_LF_RF <= 60) &&
			  !(abs(movement_left) >= 600 && abs(movement_right) >= 600)) {
				lcd_check();
		}
		
		masu++;
		
		movement_right = 0;
		movement_left  = 0;
		
		mode = mode_sel();
		
		if(mode == 4) {
			time_hips = 0;
			movement_left  = 0;
			movement_right = 0;
		
			while(!(ave_spd_L == 0 && ave_spd_R == 0 && abs(error_turn_left) <= 35 && abs(error_turn_right) <= 35)) {
				lcd_check();
			}
			
			if(need_hips) {
			
				mode = 5;
				time_hips = 0;
				movement_left  = 0;
				movement_right = 0;
			
				while(!(time_hips >= 50)) {
					lcd_check();
				}
			
				mode = 6;
				time_hips = 0;
				movement_left  = 0;
				movement_right = 0;
			
				while(!(abs(movement_left) >= 5 && abs(movement_right) >= 5)) {
					lcd_check();
				}
			}
			
			mode = mode_sel();
		}
		
		if(mode == 2 || mode == 3) {
		    time_hips = 0;
			movement_left  = 0;
			movement_right = 0;
			
			while(!(ave_spd_L == 0 && ave_spd_R == 0 && abs(error_turn_left) <= 35 && abs(error_turn_right) <= 35)) {
				lcd_check();
			}
			
			if(need_hips) {
				mode = 5;
				time_hips = 0;
				movement_left  = 0;
				movement_right = 0;
			
				while(!(time_hips >= 50)) {
					lcd_check();
				}
				mode = 6;
				time_hips = 0;
				movement_left  = 0;
				movement_right = 0;
			
				while(!(abs(movement_left) >= 5 && abs(movement_right) >= 5)) {
					lcd_check();
				}
			}
		}	
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
	
	lcd_pos(0,8);
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

void serial_number(long int value, int digit) {
	int i;
	long int base = 1;

	for(i = 0; i < digit - 1; ++i) {
		base *= 10;
	}
	
	if(value >= 0){
		for(i = 0; i < digit; ++i) {
			rs_putc(0x30 + (value / base) % 10);
			base /= 10;
		}
	}
	else{
		value = abs(value);
		rs_putc('-');
		for(i = 0; i < digit; ++i) {
			rs_putc(0x30 + (value / base) % 10);
			base /= 10;
		}
	}
}

/*
*	Function Name	: Init_Timer1
*	Tittle			: タイマー1のレジスタ設定
*	Input			:	なし
*	Output			: なし
*	Descripution	: CTCモードで割り込みを発生させる
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
	//		WGM13=0, WGM12=1, WGM11=0, WGM10=0で通常動作(データシート p.84, 表16-5の番号4)
	//		#1 = 0, #0 = 0
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
	//		分周は1/64
	//		20MHz/64 ==> 約312.5kHz(3.2us)
	//		#2 = 0, #1 = 1, #0 = 1
	//
	TCCR1B = 0b00001011;
	
	//TCNT1(Timer Counter1)
	//		タイマカウンタ(16bit)に直接アクセスできる
	//		初期値をいれる
	//
	TCNT1 = 0;
	
	//OCR1A(Timer Counter1 Output Compare A Register)
	//      いつコンペアマッチAをさせるかを設定する(16bit)
	//
	//		100ms間隔で割り込みを発生させ速度制御を行う
	//		タイマの動作クロックはメインクロックを64分周しているので321.5kHz(3.2us)
	//		10msを作りたいので
	//		(10 * 10^-3) / (3.2 * 10^-6) = 3125
	//
	//
	OCR1A = 3125;
	
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
