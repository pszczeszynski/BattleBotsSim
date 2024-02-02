#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <conio.h>

#include "hid.h"


static char get_keystroke(void);


int main()
{
	int i, r, num;
	char c, buf[64];

	// C-based example is 16C0:0480:FFAB:0200
	r = rawhid_open(1, 0x16C0, 0x0480, 0xFFAB, 0x0200);
	if (r <= 0) {
		// Arduino-based example is 16C0:0486:FFAB:0200
		r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
		if (r <= 0) {
			printf("no rawhid device found\n");
			return -1;
		}
	}
	printf("found rawhid device\n");

	while (1) {
		// check if any Raw HID packet has arrived
		num = rawhid_recv(0, buf, 64, 220);
		if (num < 0) {
			printf("\nerror reading, device went offline\n");
			rawhid_close(0);
			return 0;
		}
		if (num > 0) {
			printf("\nrecv %d bytes:\n", num);
			for (i=0; i<num; i++) {
				printf("%02X ", buf[i] & 255);
				if (i % 16 == 15 && i < num-1) printf("\n");
			}
			printf("\n");
		}
		// check if any input on stdin
		while ((c = get_keystroke()) >= 32) {
			printf("\ngot key '%c', sending...\n", c);
			buf[0] = c;
			for (i=1; i<64; i++) {
				buf[i] = 0;
			}
			rawhid_send(0, buf, 64, 100);
		}
	}
}

static char get_keystroke(void)
{
	if (_kbhit()) {
		char c = _getch();
		if (c >= 32) return c;
	}
	return 0;
}


