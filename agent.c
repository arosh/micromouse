#define TEST 1

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "agent.h"
#include "sensor.h"

#define gbi(byte,pos) (((byte) >> (pos)) & UINT8_C(1))
#define sbi(byte,pos) ((byte) |= (UINT8_C(1) << (pos)))
#define cbi(byte,pos) ((byte) &= ~(UINT8_C(1) << (pos)))

#define min(a,b) ((a) < (b) ? (a) : (b))

static const int size = MAP_SIZE;
// ceil(size*size*4/8)
static uint8_t wall[(MAP_SIZE*MAP_SIZE*4+8-1)/8];

static inline uint8_t wall_get(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	return gbi(wall[hash / 8], hash % 8);
}

static inline void wall_set(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	sbi(wall[hash / 8], hash % 8);
}

static inline void wall_reset(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	cbi(wall[hash / 8], hash % 8);
}

// ceil(size*size/8)
static uint8_t visible[(MAP_SIZE*MAP_SIZE+8-1)/8];

static inline uint8_t visible_get(const int y, const int x) {
	const int hash = y*size + x;
	return gbi(visible[hash / 8], hash % 8);
}

static inline void visible_set(const int y, const int x) {
	const int hash = y*size + x;
	sbi(visible[hash / 8], hash % 8);
}

static inline void visible_reset(const int y, const int x) {
	const int hash = y*size + x;
	cbi(visible[hash / 8], hash % 8);
}

static inline uint8_t check_pos(const int y, const int x) {
	return 0 <= y && y < size && 0 <= x && x < size;
}

static inline uint8_t check_goal(const int y, const int x) {
	return GOAL_Y <= y && y < GOAL_Y + GOAL_H && GOAL_X <= x && x < GOAL_X + GOAL_W;
}

static const int dy[] = { -1, 0, 1, 0 };
static const int dx[] = { 0, -1, 0, 1 };
static uint8_t d[MAP_SIZE][MAP_SIZE][4];

// ceil(size*size/8)
static uint8_t vis[(MAP_SIZE*MAP_SIZE*4+8-1)/8];

static inline uint8_t vis_get(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	return gbi(vis[hash / 8], hash % 8);
}

static inline void vis_set(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	sbi(vis[hash / 8], hash % 8);
}

static inline void vis_reset(const int y, const int x, const int k) {
	const int hash = y*size*4 + x*4 + k;
	cbi(vis[hash / 8], hash % 8);
}

static int curY, curX, dir;

static uint8_t can_move[MAP_SIZE][MAP_SIZE][4];
static void prepare_graph(void);

void agent_init(void) {
	// 壁の初期化
	memset(wall, 0, sizeof(wall));
	for(int i = 0; i < size; ++i) {
		// 左
		wall_set(i, 0, 1);
		// 右
		wall_set(i, size - 1, 3);
		// 上
		wall_set(0, i, 0);
		// 下
		wall_set(size - 1, i, 2);
	}

	// 座標関係
	curY = START_Y;
	curX = START_X;
	dir  = START_DIR;

	// 探索関係
	memset(visible, 0, sizeof(visible));
}

// to_goalがtrueのとき、ゴールまでの経路を探索する
// to_goalがfalseのとき、未探索の地点までの経路を探索する
enum action_t agent_explore(const bool to_goal) {
	if(to_goal == false) {
		for(int y = 0; y < size; ++y) {
			for(int x = 0; x < size; ++x) {
				vis_reset(y, x, 0);
				// 未探索の地点をゴールと設定
				if(visible_get(y, x) == false) {
					d[y][x][0] = 0;
				}
				else {
					d[y][x][0] = UINT8_MAX;
				}
			}
		}
	}
	else {
		for(int y = 0; y < size; ++y) {
			for(int x = 0; x < size; ++x) {
				vis_reset(y, x, 0);
				if(check_goal(y, x)) {
					d[y][x][0] = 0;
				}
				else {
					d[y][x][0] = UINT8_MAX;
				}
			}
		}
	}

	// ダイクストラ開始
	while(true) {
		int vy = -1, vx = -1;

		for (int y = 0; y < size; ++y) {
			for (int x = 0; x < size; ++x) {
				if (vis_get(y, x, 0) == false && ((vy == -1 && vx == -1) || d[y][x][0] < d[vy][vx][0])) {
					vy = y;
					vx = x;
				}
			}
		}

		// 未踏の地が無かったら、探索終了
		if ((vy == -1 && vx == -1) || d[vy][vx][0] == UINT8_MAX) {
			break;
		}

		vis_set(vy, vx, 0);

		for (int k = 0; k < 4; k++)
		{
			if (wall_get(vy, vx, k) == false)
			{
				int ny = vy + dy[k];
				int nx = vx + dx[k];
				d[ny][nx][0] = min(d[ny][nx][0], d[vy][vx][0] + 1);
			}
		}
	}

	if(wall_get(curY, curX, (dir + 0) % 4) == false &&
			d[curY + dy[(dir + 0) % 4]][curX + dx[(dir + 0) % 4]][0] == d[curY][curX][0] - 1) {
		// GoForward
		curY += dy[dir];
		curX += dx[dir];
		return GO_FORWARD;
	}
	
	if(wall_get(curY, curX, (dir + 1) % 4) == false &&
			d[curY + dy[(dir + 1) % 4]][curX + dx[(dir + 1) % 4]][0] == d[curY][curX][0] - 1) {
		// TurnLeft
		dir = (dir + 1) % 4;
		return TURN_LEFT;
	}

	if(wall_get(curY, curX, (dir + 3) % 4) == false &&
			d[curY + dy[(dir + 3) % 4]][curX + dx[(dir + 3) % 4]][0] == d[curY][curX][0] - 1) {
		// TurnRight
		dir = (dir + 3) % 4;
		return TURN_RIGHT;
	}

	if(wall_get(curY, curX, (dir + 2) % 4) == false &&
			d[curY + dy[(dir + 2) % 4]][curX + dx[(dir + 2) % 4]][0] == d[curY][curX][0] - 1) {
		// TurnLeft
		// 後ろを向くのが最適解の場合は、とりあえず左を向いておいて、次のステップに任せる
		return TURN_LEFT;
	}

	return NO_OPERATION;
}

static void prepare_graph(void) {
	// いもす法
	int s;

	for (int y = 0; y < size; y++)
	{
		// →
		s = 0;
		can_move[y][0][1] = s;
		for (int x = 1; x < size; x++)
		{
			if (wall_get(y, x, 1) == false)
			{
				s++;
			}
			else
			{
				s = 0;
			}
			can_move[y][x][1] = s;
		}

		// ←
		s = 0;
		can_move[y][size - 1][3] = s;
		for (int x = size - 2; x >= 0; x--)
		{
			if (wall_get(y, x, 3) == false)
			{
				s++;
			}
			else
			{
				s = 0;
			}
			can_move[y][x][3] = s;
		}

	}

	for (int x = 0; x < size; x++)
	{
		// ↓
		s = 0;
		can_move[0][x][0] = s;
		for (int y = 1; y < size; y++)
		{
			if (wall_get(y, x, 0) == false)
			{
				s++;
			}
			else
			{
				s = 0;
			}
			can_move[y][x][0] = s;
		}

		// ↑
		s = 0;
		can_move[size - 1][x][2] = s;
		for (int y = size - 2; y >= 0; y--)
		{
			if (wall_get(y, x, 2) == false)
			{
				s++;
			}
			else
			{
				s = 0;
			}
			can_move[y][x][2] = s;
		}
	}
}

void agent_compute_shortest_path(const bool to_start_area) {
	prepare_graph();
}

void agent_learn(void) {
	// 既に学習済みならセンサーから値を取り込まない
	if(visible_get(curY, curX)) return;

	visible_set(curY, curX);
	uint8_t sensor_data = sensor_get();
	for(int k = 0; k < 4; ++k) {
		// 後ろのセンサーは付いていないので
		if(k == 2) continue;

		int ndir = (dir + k) % 4;
		// 壁があるとき
		if(gbi(sensor_data, k)) {
			wall_set(curY, curX, ndir);
		}
		// 壁が無いとき
		else {
			wall_reset(curY, curX, ndir);
		}

		int ny = curY + dy[ndir];
		int nx = curX + dx[ndir];
		if(check_pos(ny, nx)) {
			if(gbi(sensor_data, k)) {
				wall_set(ny, nx, (ndir + 2) % 4);
			}
			else {
				wall_reset(ny, nx, (ndir + 2) % 4);
			}

			// 周囲4マスの壁の状態が分かっていたらvisibleにする処理
			if(visible_get(ny, nx) == false) {
				int n = 0;
				for(int l = 0; l < 4; ++l) {
					int my = ny + dy[l];
					int mx = nx + dx[l];
					if(check_pos(my, mx) == false || visible_get(my, mx)) {
						++n;
					}
				}
				if(n == 4) {
					visible_set(ny, nx);
				}
			}
		}
	}
}

#if TEST
uint8_t sensor_get(void) {
	bool w[3][3][4] = {
		{ { 1, 1, 1, 0 }, { 1, 0, 1, 1 }, { 1, 1, 0, 1 } },
		{ { 1, 1, 0, 0 }, { 1, 0, 1, 0 }, { 0, 0, 1, 1 } },
		{ { 0, 1, 1, 1 }, { 1, 1, 1, 0 }, { 1, 0, 1, 1 } }
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
		enum action_t act = agent_explore(true);
		bool end = false;
		switch(act) {
			case GO_FORWARD:
				puts("GO_FORWARD");
				break;

			case TURN_LEFT:
				puts("TURN_LEFT");
				break;

			case TURN_RIGHT:
				puts("TURN_RIGHT");
				break;

			case NO_OPERATION:
				puts("NO_OPERATION");
				end = true;
				break;
		}
		if(end) break;
		char c = getchar();
	}
	return 0;
}
#endif
// vim: noet ts=4 sw=4 sts=0
