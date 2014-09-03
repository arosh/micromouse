#define TEST 1

#include <string.h>
#include <stdint.h>
#include "agent.h"
#include "sensor.h"

#define gbi(byte,pos) (((byte) >> (pos)) & UINT8_C(1))
#define sbi(byte,pos) ((byte) |= (UINT8_C(1) << (pos)))
#define cbi(byte,pos) ((byte) &= ~(UINT8_C(1) << (pos)))

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

static const int dy[] = { -1, 0, 1, 0 };
static const int dx[] = { 0, -1, 0, 1 };
static uint8_t d[MAP_SIZE][MAP_SIZE];

// ceil(size*size/8)
static uint8_t vis[(MAP_SIZE*MAP_SIZE+8-1)/8];

static inline uint8_t vis_get(const int y, const int x) {
	const int hash = y*size + x;
	return gbi(vis[hash / 8], hash % 8);
}

static inline void vis_set(const int y, const int x) {
	const int hash = y*size + x;
	sbi(vis[hash / 8], hash % 8);
}

static inline void vis_reset(const int y, const int x) {
	const int hash = y*size + x;
	cbi(vis[hash / 8], hash % 8);
}

static int curY, curX, dir;

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

enum action_t agent_explore(void) {
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
			if(!visible_get(ny, nx)) {
				int n = 0;
				for(int l = 0; l < 4; ++l) {
					int my = ny + dy[l];
					int mx = nx + dx[l];
					if(!check_pos(my, mx) || visible_get(my, mx)) {
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
	return 0;
}
#include <stdio.h>
int main(void) {
	agent_init();
	for(int y = 0; y < size; ++y) {
		for(int x = 0; x < size; ++x) {
			putchar(' ');
			putchar(wall_get(y,x,0) ? '-' : ' ');
			putchar(' ');
		}
		putchar('\n');
		for(int x = 0; x < size; ++x) {
			putchar(wall_get(y,x,1) ? '|' : ' ');
			putchar(' ');
			putchar(wall_get(y,x,3) ? '|' : ' ');
		}
		putchar('\n');
		for(int x = 0; x < size; ++x) {
			putchar(' ');
			putchar(wall_get(y,x,2) ? '-' : ' ');
			putchar(' ');
		}
		putchar('\n');
	}
	return 0;
}
#endif
// vim: noet ts=4 sw=4 sts=0
