#include "Hymechan.h"
#include "Arduino.h"

#ifndef fl_laser
#define fl_laser 0x52 // front & left laser address
#define fr_laser 0x54 // front & right laser address
#define ll_laser 0x56 // left & left laser address
#define lr_laser 0x58 // left & right laser address
#include <Wire.h>     // I2C Library
#endif

void laser_pin() {
	Wire.begin();
}

void LaserSensorRead(unsigned char addr, unsigned char* datbuf, unsigned char cnt, int port) {
    unsigned short result = 0;
    Wire.beginTransmission(port);
    Wire.write(byte(addr));
    Wire.endTransmission();
    Wire.requestFrom(port, (int)cnt);
    if (cnt <= Wire.available()) {
        *datbuf++ = Wire.read();
        *datbuf++ = Wire.read();
    }
}

int ReadDistance(int port)
{
    unsigned short distance;
    unsigned char i2c_rx_buf[2];
    //Read 2 bytes from 0x00
    LaserSensorRead(0x00, i2c_rx_buf, 2, port);

    //Merge two bytes into one
    // i2c_rx_buf[0] : Upper byte
    // i2c_rx_buf[1] : Lower byte
    distance = i2c_rx_buf[0];
    distance = distance << 8; //Left shift bits
    distance |= i2c_rx_buf[1];

    //50ms delay
    delay(30);
    return distance;
}