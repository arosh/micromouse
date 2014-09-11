#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

// #0に前の壁の有無、#1に左の壁の有無、#3に右の壁の有無を入れてください
uint8_t sensor_get(void);

#endif /* SENSOR_H_ */
