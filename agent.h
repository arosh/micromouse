#ifndef AGENT_H_
#define AGENT_H_

#include <stdint.h>

// マップに関するパラメータ
#define MAP_SIZE 16

#define START_X 0
#define START_Y 15
// 方向は 上:0 左:1 下:2 右:3 で指定
#define START_DIR 0

#define GOAL_X 7
#define GOAL_Y 7
#define GOAL_W 2
#define GOAL_H 2

enum action_t {
  GO_FORWARD, TURN_LEFT, TURN_RIGHT, NO_OPERATION
};

// AIの初期化
void agent_init(void);

// 壁の状態を入力する (1マス移動したら実行すること)
void agent_learn(void);

// 探索走行のために、次に行うべきアクションを考える
enum action_t agent_explore(const bool adachi);

void agent_compute_shortest_path(const bool to_start_area);

#endif /* AVR_LCD_H_ */
