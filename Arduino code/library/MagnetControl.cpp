#include "Hymechan.h"
#include "Arduino.h"

#ifndef l_mag
#define l_mag 26
#define f_mag 27
#endif

void magnet_pin() {
	pinMode(l_mag, OUTPUT);
	pinMode(f_mag, OUTPUT);
	digitalWrite(l_mag, LOW);
	digitalWrite(f_mag, LOW);
}

void magnet(bool num, bool truth) {
	if (num) { //turn on left magnet
		digitalWrite(l_mag, truth);
	} else { //turn on right magnet
		digitalWrite(f_mag, truth);
	}
}