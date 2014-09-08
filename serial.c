// TODO コメントを書くこと！
void Init_Serial(void) {
	UBRR0  = 129;
	UCSR0A = 0b00000000;
	UCSR0B = 0b00011000;
	UCSR0C = 0b00000110;
}

static void rs_putc (const char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0); //UDREビットが1になるまで待つ
	UDR0 = c;
}

void rs_puts (const char *st)
{
	while (*st) {
		rs_putc (*st);
		if (*(st++) == '\n') rs_putc('\r');
	}
}

void serial_number(const long int value, const int digit) {
	int i;
	long int base = 1;

	for(i = 0; i < digit - 1; ++i) {
		base *= 10;
	}

	if(value < 0) {
		rs_putc('-');
		value *= -1;
	}

	for(i = 0; i < digit; ++i) {
		rs_putc(0x30 + (value / base) % 10);
		base /= 10;
	}
}
// vim: noet ts=4 sw=4 sts=0
