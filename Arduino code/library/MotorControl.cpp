#include "Hymechan.h"
#include "Arduino.h"

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

/////////////// motor pinMode /////////////////
void motor_pin() {
	pinMode(lf_dir, OUTPUT);
	pinMode(rf_dir, OUTPUT);
	pinMode(lb_dir, OUTPUT);
	pinMode(rb_dir, OUTPUT);
	pinMode(lf_v, OUTPUT);
	pinMode(rf_v, OUTPUT);
	pinMode(lb_v, OUTPUT);
	pinMode(rb_v, OUTPUT);
}
///////////////////////////////////////////////

/////////// ordinary motor Control ////////////
//l = left speed, r = right speed
//-- => go backwards
//-+ => turn ccw
//+= => turn cw
//++ => go forward
void linearORturn_move(int l, int r) {
    if (l < 0) {
        l = -l;
        if (r < 0) {
            r = -r;
            //move backwards
            digitalWrite(lf_dir, LOW);
            digitalWrite(lb_dir, LOW);
            digitalWrite(rf_dir, LOW);
            digitalWrite(rb_dir, LOW);
        } else {
            //turn ccw
            digitalWrite(lf_dir, LOW);
            digitalWrite(lb_dir, LOW);
            digitalWrite(rf_dir, HIGH);
            digitalWrite(rb_dir, HIGH);
        }
    } else {
        if (r < 0) {
            r = -r;
            //turn cw
            digitalWrite(lf_dir, HIGH);
            digitalWrite(lb_dir, HIGH);
            digitalWrite(rf_dir, LOW);
            digitalWrite(rb_dir, LOW);
        } else {
            //move forward
            digitalWrite(lf_dir, HIGH);
            digitalWrite(lb_dir, HIGH);
            digitalWrite(rf_dir, HIGH);
            digitalWrite(rb_dir, HIGH);
        }
    }
    analogWrite(lf_v, l);
    analogWrite(lb_v, l);
    analogWrite(rf_v, r);
    analogWrite(rb_v, r);
}
///////////////////////////////////////////////

//////////// mecanum wheel Control ////////////
void lateral_move(int v) {
    if (v < 0) { // left
        v = -v;
        digitalWrite(lf_dir, LOW);
        digitalWrite(lb_dir, HIGH);
        digitalWrite(rf_dir, HIGH);
        digitalWrite(rb_dir, LOW);
    }
    else {     // right
        digitalWrite(lf_dir, HIGH);
        digitalWrite(lb_dir, LOW);
        digitalWrite(rf_dir, LOW);
        digitalWrite(rb_dir, HIGH);
    }
    analogWrite(lf_v, v);
    analogWrite(lb_v, v);
    analogWrite(rf_v, v);
    analogWrite(rb_v, v);
}
///////////////////////////////////////////////

void diag_move(int v, uint8_t ForB) {
    //F == 1, B == 0, -v == left, v = right
    if (v < 0) {
        v = -v;
        if (ForB) {
            digitalWrite(lb_dir, HIGH);
            digitalWrite(rf_dir, HIGH);
        } else {
            digitalWrite(lb_dir, LOW);
            digitalWrite(rf_dir, LOW);
        }
        analogWrite(lf_v, 0);
        analogWrite(lb_v, v);
        analogWrite(rf_v, v);
        analogWrite(rb_v, 0);
    } else {
        if (ForB) {
            digitalWrite(lf_dir, HIGH);
            digitalWrite(rb_dir, HIGH);
        } else {
            digitalWrite(lf_dir, LOW);
            digitalWrite(rb_dir, LOW);
        }
        analogWrite(lf_v, v);
        analogWrite(lb_v, 0);
        analogWrite(rf_v, 0);
        analogWrite(rb_v, v);
    }
}