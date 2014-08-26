#ifndef AVR_TOOLS_H_
#define AVR_TOOLS_H_

// F_CPUを何回も書くとバグの原因になるかもしれないので、
// 一箇所に固めた
#define F_CPU 20000000
#include <util/delay.h>

#include <avr/sfr_defs.h>
// byteのposビット目をオンにする
#define sbi(byte,pos) (byte |= _BV(pos))
// byteのposビット目をオフにする
#define cbi(byte,pos) (byte &= ~_BV(pos))

#endif /* AVR_TOOLS_H_ */
