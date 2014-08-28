#define TEST 1

#include <string.h>
#include <stdint.h>
#include "agent.h"
#include "sensor.h"

static const int size = MAP_SIZE;
// ceil(size*size*4/8)
static uint8_t wall[(MAP_SIZE*MAP_SIZE*4+8-1)/8];

static inline uint8_t wall_get(const int y, const int x, const int k) {
  return (wall[(y*size*4 + x*4 + k) / 8] >> ((y*size*4 + x*4 + k) % 8)) & UINT8_C(1);
}

static inline void wall_set(const int y, const int x, const int k) {
  wall[(y*size*4 + x*4 + k) / 8] |=  (UINT8_C(1) << ((y*size*4 + x*4 + k) % 8));
}

static inline void wall_reset(const int y, const int x, const int k) {
  wall[(y*size*4 + x*4 + k) / 8] &= ~(UINT8_C(1) << ((y*size*4 + x*4 + k) % 8));
}

// ceil(size*size/8)
static uint8_t visible[(MAP_SIZE*MAP_SIZE+8-1)/8];

static inline uint8_t visible_get(const int y, const int x) {
  return (visible[(y*size + x) / 8] >> ((y*size + x) % 8)) & UINT8_C(1);
}

static inline void visible_set(const int y, const int x) {
  visible[(y*size + x) / 8] |=  (UINT8_C(1) << ((y*size + x) % 8));
}

static inline void visible_reset(const int y, const int x) {
  visible[(y*size + x) / 8] &= ~(UINT8_C(1) << ((y*size + x) % 8));
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
  return (vis[(y*size + x) / 8] >> ((y*size + x) % 8)) & UINT8_C(1);
}

static inline void vis_set(const int y, const int x) {
  vis[(y*size + x) / 8] |=  (UINT8_C(1) << ((y*size + x) % 8));
}

static inline void vis_reset(const int y, const int x) {
  vis[(y*size + x) / 8] &= ~(UINT8_C(1) << ((y*size + x) % 8));
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
  if(get_visible(curY, curX)) return;

  set_visible(curY, curX);
  uint8_t sensor_data = sensor_get();
  for(int k = 0; k < 4; ++k) {
    // 後ろのセンサーは付いていないので
    if(k == 2) continue;

    int ndir = (dir + k) % 4;
    if((sensor_data >> k) & 1) {
      set_wall(curY, curX, ndir);
    }
    else {
      reset_wall(curY, curX, ndir);
    }

    int ny = curY + dy[ndir];
    int nx = curX + dx[ndir];
    if(check_pos(ny, nx)) {
      wall[ny, nx, (ndir + 2) % 4] = wall[curY, curX, ndir];

      if(get_visible(ny, nx) == false) {
        int n = 0;
        for(int l = 0; l < 4; ++l) {
          int my = ny + dy[l];
          int mx = nx + dx[l];
          if(check_pos(my, mx) == false || get_visible(my, mx)) {
            ++n;
          }
        }
        if(n == 4) {
          set_visible(ny, nx);
        }
      }
    }
  }
}

#if TEST
#include <stdio.h>
int main(void) {
  agent_init();
  return 0;
}
#endif
