bool LaserTooClose() {
  updateLaser();
  if (laser_LL < LASER_MARGIN) return true;
  if (laser_LR < LASER_MARGIN) return true;
  if (laser_FL < LASER_MARGIN) return true;
  if (laser_FR < LASER_MARGIN) return true;
  return false;
}

void updateLaser() {
  if (ACTUAL_ID == 0) {
    return;
  } else {
    laser_LL = ReadDistance(ll_laser);
    laser_LR = ReadDistance(lr_laser);
    laser_FL = ReadDistance(fl_laser);
    laser_FR = ReadDistance(fr_laser);
  }
}

void sendLaserData() {
  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.print(" laser_LL : ");
  Serial.print(laser_LL);
  Serial.print(" laser_LR : ");
  Serial.print(laser_LR);
  Serial.print(" laser_FL : ");
  Serial.print(laser_FL);
  Serial.print(" laser_FR : ");
  Serial.println(laser_FR);
}

//Compares either front lasers or left lasers for robot alignment
//ForL = Front or Left & False = Front, True = Left
//Front threshold is 300, Left threshold is 420
//Returns -2 when left linear movement is needed
//Returns -1 when left turn is needed
//Returns  0 when aligned
//Returns  1 when right turn is needed
//Returns  2 when right linear movement is needed
//Returns  3 when forward movement is needed
//Returns  9 when it is aligned and close enough to another robot
int8_t isLaserAligned(bool ForL) {
  if (ACTUAL_ID == 0) {
    return 0;
  } else {
    int leftLaserValue = 0;
    int rightLaserValue = 0;

    updateLaser();
    if (ForL) {
      //LEFT Laser
      leftLaserValue = laser_LL;
      rightLaserValue = laser_LR;
      
      if (leftLaserValue >= LASER_LEFT_THRESHOLD && rightLaserValue >= LASER_LEFT_THRESHOLD) {
        return 3;
      } else if (leftLaserValue >= LASER_LEFT_THRESHOLD && rightLaserValue < LASER_LEFT_THRESHOLD) {
        return 2;
      } else if (leftLaserValue < LASER_LEFT_THRESHOLD && rightLaserValue >= LASER_LEFT_THRESHOLD) {
        return -2;
      } else if ((abs(leftLaserValue - rightLaserValue) < LASER_MARGIN) && leftLaserValue < LASER_MARGIN && rightLaserValue < LASER_MARGIN) {
        return 9;
      } else if ((abs(leftLaserValue - rightLaserValue) < LASER_MARGIN) && leftLaserValue > LASER_ESCAPE_LOOP_THLD && rightLaserValue < LASER_ESCAPE_LOOP_THLD) {
        return 1;
      } else if ((abs(leftLaserValue - rightLaserValue) < LASER_MARGIN) && leftLaserValue < LASER_ESCAPE_LOOP_THLD && rightLaserValue > LASER_ESCAPE_LOOP_THLD) {
        return -1;
      } else if (leftLaserValue - rightLaserValue > 30) {
        return 1;
      } else if (rightLaserValue - leftLaserValue > 30) {
        return -1;
      } else {
        return 0;
      }
    } else {
      //RIGHT Laser
      leftLaserValue = laser_FL;
      rightLaserValue = laser_FR;

      if (leftLaserValue >= LASER_FRONT_THRESHOLD && rightLaserValue >= LASER_FRONT_THRESHOLD) {
        return 3;
      } else if (leftLaserValue >= LASER_FRONT_THRESHOLD && rightLaserValue < LASER_FRONT_THRESHOLD) {
        return 2;
      } else if (leftLaserValue < LASER_FRONT_THRESHOLD && rightLaserValue >= LASER_FRONT_THRESHOLD) {
        return -2;
      } else if ((abs(leftLaserValue - rightLaserValue) < LASER_MARGIN) && leftLaserValue < LASER_MARGIN && rightLaserValue < LASER_MARGIN) {
        return 9;
      } else if (abs(leftLaserValue - rightLaserValue) < 20) {
        return 0;
      } else if (leftLaserValue > rightLaserValue) {
        return 1;
      } else if (leftLaserValue < rightLaserValue) {
        return -1;
      } else {
        return 0;
      }
    }
  }
}
