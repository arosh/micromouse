#ifndef PID_H_
#define PID_H_
typedef struct {
  float Kp, Ki, Kd;
  const float *current_value, *target_value;
  float error_integr; // 積分値(I制御のために)
  float error_prev; // 前回の値(D制御のため)
} typedef Pid;
#endif /* PID_H_ */
