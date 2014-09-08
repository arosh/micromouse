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
#include "interrupt.h"

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

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
volatile int sensor_distance_AVE_LF_RF;

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
float ave_spd_R = 0;
float ave_spd_L = 0;

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

volatile char mode = 1;
volatile int time_sips;
volatile char old_mode = 1;
volatile char time_tyoi;

void ave_speed(void)
{
	static long int old_pulse_R = 0;
	static long int old_pulse_L = 0;
	long int now_pulse_R = Right_RotaryEncorder_val;
	long int now_pulse_L = Left_RotaryEncorder_val;
	int i;
	
	//この割り込み関数は10msごとに読み込まれるのでパルスカウントの差を10ms[0.01s]で割ることでパルス速度[pulse/s]を求めることができる
	spd_R[time] = (now_pulse_R - old_pulse_R) / 0.01;
	spd_L[time] = (now_pulse_L - old_pulse_L) / 0.01;
	old_pulse_R = now_pulse_R;
	old_pulse_L = now_pulse_L;
	
	time++;
	if(time == 3){
		time = 0;
	}
	
	for(i = 0;i < 3;i++){
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
	int sensor_distance_AVE_LF_RF = ((int)sensor_distance_LF+(int)sensor_distance_RF)/2;
}

void one_forward(void)
{
	if((movement_right <= 550) && (movement_left <= 550)){
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
	else{
		motor_brake_left();
		motor_brake_right();
		
		_delay_ms(1000);
		
		movement_left  = 0;
		movement_right = 0;
	}
}

void forward_hips(void)
{
	if((movement_right <= 50) && (movement_left <= 50)){
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
	else{
		motor_brake_left();
		motor_brake_right();
		
		_delay_ms(1000);
		
		movement_left  = 0;
		movement_right = 0;
	}
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
	const float Kp_down_right = 0.60;
	const float Kp_down_left  = 0.60;
	
	const char prefarance_sensor_right = 40;
	const char prefarance_sensor_left  = 40;
	
	int error_sensor_right;
	int error_sensor_left;
	
	int P_control_down_right;
	int P_control_down_left;
	
	error_sensor_right = abs(prefarance_sensor_right - sensor_distance_RF);
	error_sensor_left  = abs(prefarance_sensor_left  - sensor_distance_LF);
	
	P_control_down_right = (int)(Kp_down_right * error_sensor_right);
	P_control_down_left  = (int)(Kp_down_left  * error_sensor_left);
	
	//motor_right(P_control_down_right);
	//motor_left(P_control_down_left);
	
	motor_brake_right();
	motor_brake_left();
	
}

void turn_right(void)
{
	volatile int turn_select = 1;	// 1なら右回転	-1なら左回転
	
	const float Kp_turn_right = 0.50;
	const float Kp_turn_left  = 0.55;
	const float Kd_turn_right = 0.1;
	const float Kd_turn_left  = 0.1;
	
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
	
	const float Kp_turn_right = 0.50;
	const float Kp_turn_left  = 0.55;
	const float Kd_turn_right = 0.1;
	const float Kd_turn_left  = 0.1;
	
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
	const float Kp_hips_R = 0.1;
	const float Kp_hips_L = 0.1;
	
	int prefer_hips_R = 135;
	int prefer_hips_L = 135;
	
	int error_hips_R;
	int error_hips_L;
	
	int P_control_hips_R;
	int P_control_hips_L;
	
	motor_left(-45);
	motor_right(-45);
	time_sips++;
}


int mode_sel(void)
{
	bool wall_r = sensor_distance_R < 80;
	bool wall_l = sensor_distance_L < 80;
	bool wall_f = (sensor_distance_LF + sensor_distance_RF) / 2 < 35;
	
	if(wall_f == false) {
		return 1;
	}
	else if(wall_r == false) {
		prefer_turn_flag = 1;
		return 2;
	}
	else if(wall_l == false) {
		prefer_turn_flag = 1;
		return 3;
	}
	else {
		//180turn
		prefer_turn_flag = 1;
		return 4;
	}
}

// センサ用割り込み
ISR(TIMER1_COMPA_vect){
	
	ave_speed();		//速度の平均をとる
	sensor_convert();	//AD変換値を距離[mm]に変換
	
	//char old_mode = 0;
	
	if(((ave_spd_R == 0) && (ave_spd_L == 0) && (abs(error_turn_left) <= 35) && (abs(error_turn_right) <= 35)) ||
		((ave_spd_R == 0) && (ave_spd_L == 0) && (time_sips >= 40)) ||
		 ((ave_spd_L == 0) && (ave_spd_R == 0) && (mode == 1)) || ((ave_spd_L == 0) && (ave_spd_R == 0) && (mode == 6))){

		time_sips = 0;
		
		mode = mode_sel();
		
		if((old_mode == 2) || (old_mode == 3) || (old_mode == 4)){
			mode = 5;
		}
		if(old_mode == 5){
			mode = 6;
		}
		old_mode = mode;
	}
	
	switch(mode){
		
		case 1:
			movement_right += spd_R[time] * 0.01;	//速度を積分
			movement_left  += spd_L[time] * 0.01;
					
			one_forward();
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
	}
}

			

// エンコーダ用割り込み
ISR(TIMER3_COMPA_vect)
{
	static int sensor_count = 0;
	sensor_count++;
	
	if(sensor_count >= 250){
		Init_ADC_get();
		sensor_count = 0;
	}
	
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
	
	UBRR0  = 129;
	UCSR0A = 0b00000000;
	UCSR0B = 0b00011000;
	UCSR0C = 0b00000110;
	
	//AD変換レジスタ設定
	loop_until_bit_is_clear(PINB,PINB2);		//スタートスイッチ(青色)が押されるまで待機
	
	beep();										//ブザーを鳴らす
	
	sei();		//割り込み許可
	
	while(1){

		lcd_pos(0,0);
		lcd_str("mode");
		lcd_pos(0,6);
		lcd_number(mode, 1);
		lcd_pos(0,9);
		lcd_str("old");
		lcd_pos(0,13);
		lcd_number(old_mode, 1);
		lcd_pos(1,1);
		lcd_number(movement_left, 3);
		lcd_pos(1,9);
		lcd_number(movement_right, 3);
		lcd_pos(0,0);
		
		//serial_number(pulse_velocity_left, 5);
		//rs_putc(',');
		//serial_number(pulse_velocity_right, 5);
		//rs_puts("\n\r");
		//
		//print_all_sensor();
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

// vim: noet ts=4 sw=4 sts=0
