#define TEST 1

#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include "agent.h"
#include "sensor.h"

#define gbi(byte,pos) (((byte) >> (pos)) & UINT8_C(1))
#define sbi(byte,pos) ((byte) |= (UINT8_C(1) << (pos)))
#define cbi(byte,pos) ((byte) &= ~(UINT8_C(1) << (pos)))

#define min(a,b) ((a) < (b) ? (a) : (b))

static const int size = MAP_SIZE;
static bool wall[MAP_SIZE][MAP_SIZE][4];
static bool visible[MAP_SIZE][MAP_SIZE];

static const int dy[] = { -1, 0, 1, 0 };
static const int dx[] = { 0, -1, 0, 1 };
static unsigned char d[MAP_SIZE][MAP_SIZE];

static inline bool check_pos(const int y, const int x) {
	return 0 <= y && y < size && 0 <= x && x < size;
}

static inline bool check_goal(const int y, const int x) {
	return GOAL_Y <= y && y < GOAL_Y + GOAL_H && GOAL_X <= x && x < GOAL_X + GOAL_W;
}

static int curY, curX, dir;

static int queueTop, queueBottom;
static int queueX[MAP_SIZE*MAP_SIZE+1];
static int queueY[MAP_SIZE*MAP_SIZE+1];
static bool inQueue[MAP_SIZE][MAP_SIZE];

static void queueInit(void) {
	queueTop = 0;
	queueBottom = 0;
}

static void queueEnqueue(const int y, const int x) {
	queueY[queueTop] = y;
	queueX[queueTop] = x;
	queueTop = (queueTop + 1) % (MAP_SIZE*MAP_SIZE+1);
}

static void queueDequeue(int *y, int *x) {
	*y = queueY[queueBottom];
	*x = queueX[queueBottom];
	queueBottom = (queueBottom + 1) % (MAP_SIZE*MAP_SIZE+1);
}

static bool queueEmpty(void) {
	return queueBottom == queueTop;
}

void agent_init(void) {
	// 壁の初期化
	for(int y = 0; y < size; ++y) {
		for(int x = 0; x < size; ++x) {
			visible[y][x] = false;
			for(int k = 0; k < size; ++k) {
				wall[y][x][k] = false;
			}
		}
	}
	for(int i = 0; i < size; ++i) {
		// 左
		wall[i][0][1] = true;
		// 右
		wall[i][size - 1][3] = true;
		// 上
		wall[0][i][0] = true;
		// 下
		wall[size - 1][i][2] = true;
	}

	// 座標関係
	curY = START_Y;
	curX = START_X;
	dir  = START_DIR;
}

// to_goalがtrueのとき、ゴールまでの経路を探索する
// to_goalがfalseのとき、未探索の地点までの経路を探索する
enum action_t agent_explore(void) {
	queueInit();
	for(int y = 0; y < size; ++y) {
		for(int x = 0; x < size; ++x) {
			if(check_goal(y, x)) {
				d[y][x] = 0;
				inQueue[y][x] = true;
				queueEnqueue(y, x);
			}
			else {
				d[y][x] = UCHAR_MAX - 1;
				inQueue[y][x] = false;
			}
		}
	}

	while(queueEmpty() == false) {
		int vy, vx;
		queueDequeue(&vy, &vx);
		inQueue[vy][vx] = false;

		for (int k = 0; k < 4; k++) {
			if (wall[vy][vx][k] == false) {
				int ny = vy + dy[k];
				int nx = vx + dx[k];
				if(d[ny][nx] > d[vy][vx] + 1) {
					d[ny][nx] = d[vy][vx] + 1;
					if(inQueue[ny][nx] == false) {
						inQueue[ny][nx] = true;
						queueEnqueue(ny, nx);
					}
				}
			}
		}
	}

	if(wall[curY][curX][(dir + 0)%4] == false &&
			d[curY + dy[(dir + 0)%4]][curX + dx[(dir + 0)%4]] + 1 == d[curY][curX]) {
		// GoForward
		curY += dy[dir];
		curX += dx[dir];
		return GO_FORWARD;
	}

	if(wall[curY][curX][(dir + 1)%4] == false &&
			d[curY + dy[(dir + 1)%4]][curX + dx[(dir + 1)%4]] + 1 == d[curY][curX]) {
		// TurnLeft
		dir = (dir + 1) % 4;
		return TURN_LEFT;
	}

	if(wall[curY][curX][(dir + 3)%4] == false &&
			d[curY + dy[(dir + 3)%4]][curX + dx[(dir + 3)%4]] + 1 == d[curY][curX]) {
		// TurnRight
		dir = (dir + 3) % 4;
		return TURN_RIGHT;
	}

	if(wall[curY][curX][(dir + 2)%4] == false &&
			d[curY + dy[(dir + 2)%4]][curX + dx[(dir + 2)%4]] + 1 == d[curY][curX]) {
		// TurnLeft
		dir = (dir + 1) % 4;
		return TURN_LEFT;
	}

	return NO_OPERATION;
}

void agent_learn(void) {
	// 既に学習済みならセンサーから値を取り込まない
	if(visible[curY][curX]) return;

	visible[curY][curX] = true;
	uint8_t sensor_data = sensor_get();
	for(int k = 0; k < 4; ++k) {
		// 後ろのセンサーは付いていないので
		if(k == 2) continue;

		int ndir = (dir + k)%4;
		// 壁があるとき
		if(gbi(sensor_data, k)) {
			wall[curY][curX][ndir] = true;
		}

		int ny = curY + dy[ndir];
		int nx = curX + dx[ndir];
		if(check_pos(ny, nx)) {
			if(gbi(sensor_data, k)) {
				wall[ny][nx][(ndir + 2)%4] = true;
			}
		}
	}
}

#if TEST
uint8_t sensor_get(void) {
	bool w[3][3][4] = {
		{ { 1, 1, 0, 1 }, { 1, 1, 0, 0 }, { 1, 0, 1, 1 } },
		{ { 0, 1, 0, 0 }, { 0, 0, 0, 1 }, { 1, 1, 0, 1 } },
		{ { 0, 1, 1, 1 }, { 0, 1, 1, 0 }, { 0, 0, 1, 1 } }
	};
	uint8_t r = 0;
	if(w[curY][curX][dir]) sbi(r,0);
	if(w[curY][curX][(dir+1)%4]) sbi(r,1);
	if(w[curY][curX][(dir+3)%4]) sbi(r,3);
	return r;
}

#include <stdio.h>

int main(void) {
	agent_init();
	while(1) {
		agent_learn();
		enum action_t act = agent_explore();
		bool end = false;
		switch(act) {
			case GO_FORWARD:
				printf("GO_FORWARD");
				break;

			case TURN_LEFT:
				printf("TURN_LEFT");
				break;

			case TURN_RIGHT:
				printf("TURN_RIGHT");
				break;

			case NO_OPERATION:
				printf("NO_OPERATION");
				end = true;
				break;
		}
		if(end) break;
		char buf[1024];
		fgets(buf, sizeof(buf), stdin);
	}
	puts("");
	return 0;
}
#endif
// vim: noet ts=4 sw=4 sts=0
