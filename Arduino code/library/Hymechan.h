#include "Arduino.h"
#ifndef fl_laser
#define fl_laser 0x52 // front & left laser
#define fr_laser 0x54 // front & right laser
#define ll_laser 0x56 // left & left laser
#define lr_laser 0x58 // left & right laser
#include <Wire.h>
#endif

#ifndef l_mag
#define l_mag 26
#define f_mag 27
#endif

#ifndef lf_dir
/////////////////motor pin ///////////////////
#define lf_dir 22  // left forward direction
#define rf_dir 23  // right forward direction
#define lb_dir 24  // left back direction
#define rb_dir 25  // right back direction
#define lf_v 2     // left forward velocity
#define rf_v 3     // right forward velocity
#define lb_v 4     // left back velocity
#define rb_v 5     // right back velocity
//////////////////////////////////////////////
#endif

/**********************
* MotorControl function
***********************/
void motor_pin();        // initialize motor pin
void linearORturn_move(int, int);     // Moves
void lateral_move(int);        // mechanum wheel move
void diag_move(int, uint8_t); //diagonal move (speed, ForB)

/***********************
* MagnetControl function
************************/
void magnet_pin();       // initialize magnet
void magnet(bool, bool);  // use magnet

/**********************
* LaserControl function
***********************/
void laser_pin();        // initialize laser
void LaserSensorRead(unsigned char, unsigned char*, unsigned char, int); // used in ReadDistance()
int ReadDistance(int);   // returns distance value

/*********************
* GyroControl function
**********************/