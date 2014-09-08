#include "pid.h"
/*
 * 使い方
 * === 初期化 ===
 * Pid p; // 構造体を作る (制御する値それぞれについて作る)
 * // 構造体の初期化
 * // *p: 構造体のポインタ(構造体の実体は、グローバル変数かstaticで持っておく)
 * // Kp: P制御のゲイン
 * // Ki: I制御のゲイン(制御の教科書のTiと定義が全く違うので注意)
 * // Kd: D制御のゲイン
 * // P制御だけ使いたいときはKi=0.0, Kd=0.0にすれば良い (これがTiでなくKiを使う理由)
 * // current_value, target_value: 現在値と目標値を入れる変数のポインタを渡す
 * // (Pid_updateが勝手に監視)
 * Pid_init(&p, 1.23, 0.456, 0.0789, &speed, &target_speed);
 * === 初期化終了 ===
 *
 * === 制御 ==
 * motor = Pid_update(&p); // 新しい制御量が得られる
 * === 制御終了 ===
 */

void Pid_init(Pid *pid,
		const float Kp, const float Ki, const float Kd,
		const float *current_value, const float *target_value) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->current_value = current_value;
	pid->target_value = target_value;
	pid->error_integr = 0.0F;
	pid->error_prev = *target_value - *current_value; // current_valueが初期化されていなかったらどうしよう
}

float Pid_update(Pid *pid) {
	// 偏差
	const float error = *pid->target_value - *pid->current_value;
	// 偏差の積分値
	pid->error_integr += error;
	// 偏差の微分値
	const float diff = error - pid->error_prev;
	pid->error_prev = error;

	return pid->Kp * error
		 + pid->Ki * pid->error_integr
		 + pid->Kd * diff;
}
// vim: noet ts=4 sw=4 sts=0
