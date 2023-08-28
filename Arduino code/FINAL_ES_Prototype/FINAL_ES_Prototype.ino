#include <Wire.h>
#include <Hymechan.h>
/************************* Hymechan.h functions ************************
  Basic Movement: linearORturn_move (leftspeed, rightspeed);
  -- => go backwards
  -+ => turn ccw
  += => turn cw
  ++ => go forward

  Lateral Movement: lateral_move(speed); positive speed = right, negative is left
  Diagonal Movement: diag_move(speed, ForB); positive speed = right
  Distance Measure: ReadDistance(Laser sensor name);
      returns int,
      Naming: ll_laser(left laser of the left side),
              fl_laser(left laser of the front)
  Magnet: magnet(bool ForL, bool onORoff); Front = 1, Left = 0, on = 1 off = 0
************************************************************************/

//anchor coordinates
#define anchor2_X 4.0
#define anchor3_Y 4.0

#define motorSpd 150 //moving delay is calculated based on 150 speed. change delay accordingly if speed change is needed
#define halfSpd 75
#define turnSpd 45

#define LEFT true
#define FRONT false
#define LASER_MARGIN 20 //unit : mm
#define LASER_ESCAPE_LOOP_THLD 60
#define LASER_LEFT_THRESHOLD 420
#define LASER_FRONT_THRESHOLD 300

#define ACTUAL_ID 3

uint8_t positional_ID = 0;  //ID given to each robot depending on its position prior to the movement

//robot is 42cm by 30cm
//Xcenter and Ycenter is not const so that robot can be deployed to -
//different locations within the boundaries
//CAUTION : Make sure to leave some room for error when assigning limits (around 0.15m to be safe)
float Xcenter = 200;
float Ycenter = 200;
const float rectangle_assemble_coordinates[4][2] = {{Xcenter - 21, Ycenter + 30}, {Xcenter + 21, Ycenter + 30},
                                                    {Xcenter - 21, Ycenter}, {Xcenter + 21, Ycenter}};
const float vertical_assemble_coordinates[4][2] = {{Xcenter, Ycenter + 60}, {Xcenter, Ycenter + 30},
                                                   {Xcenter, Ycenter}, {Xcenter, Ycenter - 30}};
const float horizontal_assemble_coordinates[4][2] = {{Xcenter - 63, Ycenter + 15}, {Xcenter - 21, Ycenter + 15},
                                                     {Xcenter + 21, Ycenter + 15}, {Xcenter + 63, Ycenter + 15}};
const float original_coords[4][2] = {{30, anchor3_Y * 100 - 60}, {anchor2_X * 100 - 60, anchor3_Y * 100 - 60},
                                     {30, 60}, {anchor2_X * 100 - 60, 60}};

//Anchors' position coordinates
float anchor1[2] = {0.0, 0.0};
float anchor2[2] = {anchor2_X, 0.0};
float anchor3[2] = {0.0, anchor3_Y};

float current_pos_tril[2] = {0, 0}; //current coords calculated from trilateration
float current_pos_intersect[2] = {0, 0}; //current coords calculated from intersection of 2 lines
float delta_to_goal[3] = {0, 0, 0};
float goal_coords[3] = {0, 0, 0};

//Distance variables used to get distance values from UWB
float distance[3];
uint8_t distanceIndex = 0;

//General Variables
bool rxCheck[3] = {false, false, false};
bool allDistanceReceived = false;
bool isDistanceDataReceived = false;
String distanceString = "";
float distanceValue = 0.0;

bool ReadyToMove[4] = {false, false, false, false};
bool ALL_RTM = false;

bool rectangle = false;
bool vertical = false;
bool horizontal = false;

int laser_LL = 0;
int laser_LR = 0;
int laser_FL = 0;
int laser_FR = 0;

int8_t mode = -1; //default = -1
double pwmDelay_x = 0;
double pwmDelay_y = 0;
double pwmDelay_diag = 0;
double pwmDelay_turn = 0;

void setup() {
  motor_pin();
  magnet_pin();
  laser_pin();

  magnet(0,1);  //Right magnet off
  magnet(1,1);  //Left magnet off

  Serial.begin(115200);   //RasberryPi
  Serial1.begin(115200);  //DWM1000
  switchToAnchor(); //Initialize every robot as an anchor
}

/*
   mode
   0 = rectangle assembly
   1 = vertical assembly
   2 = horizontal assembly
   3 = front or back move
   4 = lateral move
   5 = disassembly
   6 = stationary turn
*/
void loop() {
  int tempSpd = 0;
  int ForB = 0;
  bool LorF = 0;
  int8_t alignmentCode = 0;
  switch (mode) {
    case 0:
      //Wait for other robots to finish calculation
      for (int i = 0; i < ACTUAL_ID; i++) {
        while (!ReadyToMove[i]) rxCommand();
      }

      //Calculate delay based on current position using trilateration
      switchToTag();
      delay(7000);          //Delay is to ensure correct distance data is receieved
      calcCurrentPos();     //can be removed if seem unnecessary
      calcDelta();          //use betterdelay() if needed
      convDtoDelay();
      switchToAnchor();
      waitForAllDone();

      if      (ACTUAL_ID == 0) diag_move(-motorSpd, 0);
      else if (ACTUAL_ID == 1) diag_move(motorSpd, 0);
      else if (ACTUAL_ID == 2) diag_move(motorSpd, 1);
      else if (ACTUAL_ID == 3) diag_move(-motorSpd, 1);
      
      betterDelay(pwmDelay_diag);
      linearORturn_move(0, 0);
      backToDefault();

      //int8_t isLaserAligned(bool LorF)
      //Returns -2 when left linear movement is needed
      //Returns -1 when left turn is needed
      //Returns  0 when aligned
      //Returns  1 when right turn is needed
      //Returns  2 when right linear movement is needed
      //Returns  3 when forward movement is needed
      //Returns  9 when it is aligned and close enough to another robot
      //LEFT = true, FRONT = false
      updateLaser();
      sendLaserData();
      if (ACTUAL_ID == 1) LorF = LEFT;
      else if (ACTUAL_ID == 2) LorF = FRONT;
      else if (ACTUAL_ID == 3) LorF = LEFT;
      if (ACTUAL_ID != 0) {
        while (alignmentCode != 9) {
          alignmentCode = isLaserAligned(LorF);
          if (alignmentCode == -2) {
            if (LorF) linearORturn_move(-halfSpd, -halfSpd);
            else lateral_move(-halfSpd);
          } else if (alignmentCode == -1) {
            linearORturn_move(-halfSpd, halfSpd);
          } else if (alignmentCode == 0) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          } else if (alignmentCode == 1) {
            linearORturn_move(halfSpd, -halfSpd);
          } else if (alignmentCode == 2) {
            if (LorF) linearORturn_move(halfSpd, halfSpd);
            else lateral_move(halfSpd);
          } else if (alignmentCode == 3) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          }
        }
      }

      linearORturn_move(0, 0);
      magnet(1,0);
      magnet(0,0);
      backToDefault();
    break;

    case 1:
      for (int i = 0; i < ACTUAL_ID; i++) {
        while (!ReadyToMove[i]) rxCommand();
      }

      switchToTag();
      delay(7000);
      calcCurrentPos();
      calcDelta();
      convDtoDelay();
      switchToAnchor();
      waitForAllDone();

      if (BelowGoal()) tempSpd = motorSpd;
      else tempSpd = -motorSpd;
      linearORturn_move(tempSpd, tempSpd);
      betterDelay(pwmDelay_y);
      if (LeftOfGoal()) tempSpd = motorSpd;
      else tempSpd = -motorSpd;
      lateral_move(tempSpd);
      betterDelay(pwmDelay_x);
      linearORturn_move(0, 0);
      backToDefault();

      updateLaser();
      sendLaserData();
      if (ACTUAL_ID == 1) LorF = FRONT;
      else if (ACTUAL_ID == 2) LorF = FRONT;
      else if (ACTUAL_ID == 3) LorF = FRONT;

      if (ACTUAL_ID != 0) {
        while (alignmentCode != 9) {
          alignmentCode = isLaserAligned(LorF);
          if (alignmentCode == -2) {
            if (LorF) linearORturn_move(-halfSpd, -halfSpd);
            else lateral_move(-halfSpd);
          } else if (alignmentCode == -1) {
            linearORturn_move(-halfSpd, halfSpd);
          } else if (alignmentCode == 0) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          } else if (alignmentCode == 1) {
            linearORturn_move(halfSpd, -halfSpd);
          } else if (alignmentCode == 2) {
            if (LorF) linearORturn_move(halfSpd, halfSpd);
            else lateral_move(halfSpd);
          } else if (alignmentCode == 3) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          }
        }
      }

      magnet(1,0);
      linearORturn_move(0, 0);
      backToDefault();
      break;

    case 2:
      for (int i = 0; i < ACTUAL_ID; i++) {
        while (!ReadyToMove[i]) rxCommand();
      }

      switchToTag();
      delay(7000);
      calcCurrentPos();
      calcDelta();
      convDtoDelay();
      switchToAnchor();
      waitForAllDone();

      if (LeftOfGoal()) tempSpd = motorSpd;
      else tempSpd = -motorSpd;
      lateral_move(tempSpd);
      betterDelay(pwmDelay_x);
      if (BelowGoal()) tempSpd = motorSpd;
      else tempSpd = -motorSpd;
      linearORturn_move(tempSpd, tempSpd);
      betterDelay(pwmDelay_y);
      linearORturn_move(0, 0);
      backToDefault();

      updateLaser();
      sendLaserData();
      if (ACTUAL_ID == 1) LorF = LEFT;
      else if (ACTUAL_ID == 2) LorF = LEFT;
      else if (ACTUAL_ID == 3) LorF = LEFT;
      if (ACTUAL_ID != 0) {
        while (alignmentCode != 9) {
          alignmentCode = isLaserAligned(LorF);
          if (alignmentCode == -2) {
            if (LorF) linearORturn_move(-halfSpd, -halfSpd);
            else lateral_move(-halfSpd);
          } else if (alignmentCode == -1) {
            linearORturn_move(-halfSpd, halfSpd);
          } else if (alignmentCode == 0) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          } else if (alignmentCode == 1) {
            linearORturn_move(halfSpd, -halfSpd);
          } else if (alignmentCode == 2) {
            if (LorF) linearORturn_move(halfSpd, halfSpd);
            else lateral_move(halfSpd);
          } else if (alignmentCode == 3) {
            if (LorF) lateral_move(-halfSpd);
            else linearORturn_move(halfSpd, halfSpd);
          }
        }
      }

      magnet(0,0);
      linearORturn_move(0, 0);
      backToDefault();
      break;

    case 3:
      convDtoDelay();
      Serial.println("!Starting to move");
      if      (delta_to_goal[1] >= 0) linearORturn_move(motorSpd, motorSpd);
      else if (delta_to_goal[1] <  0) linearORturn_move(-motorSpd, -motorSpd);
      betterDelay(pwmDelay_y);
      linearORturn_move(0, 0);
      backToDefault();
      break;

    case 4:
      convDtoDelay();
      Serial.println("!Starting to move");
      if      (delta_to_goal[0] >= 0) lateral_move(motorSpd);
      else if (delta_to_goal[0] <  0) lateral_move(-motorSpd);
      betterDelay(pwmDelay_x);
      linearORturn_move(0, 0);
      backToDefault();
      break;

    case 5:
      for (int i = 0; i < ACTUAL_ID; i++) {
        while (!ReadyToMove[i]) rxCommand();
      }

      switchToTag();
      delay(7000);
      calcCurrentPos();
      calcDelta();
      convDtoDelay();
      switchToAnchor();
      waitForAllDone();

      if (rectangle) {
        if (LeftOfGoal() && BelowGoal()) {
          tempSpd = motorSpd;
          ForB    = 1;
        }
        else if (LeftOfGoal())           {
          tempSpd = motorSpd;
          ForB    = 0;
        }
        else if (BelowGoal())            {
          tempSpd = -motorSpd;
          ForB    = 1;
        }
        else                             {
          tempSpd = -motorSpd;
          ForB    = 0;
        }
        diag_move(tempSpd, ForB);
        betterDelay(pwmDelay_diag);
        rectangle = false;
      } else if (vertical) {
        if (LeftOfGoal()) tempSpd = motorSpd;
        else tempSpd = -motorSpd;
        lateral_move(tempSpd);
        betterDelay(pwmDelay_x);
        if (BelowGoal()) tempSpd = motorSpd;
        else tempSpd = -motorSpd;
        linearORturn_move(tempSpd, tempSpd);
        betterDelay(pwmDelay_y);
        vertical = false;
      } else if (horizontal) {
        if (BelowGoal()) tempSpd = motorSpd;
        else tempSpd = -motorSpd;
        linearORturn_move(tempSpd, tempSpd);
        betterDelay(pwmDelay_y);
        if (LeftOfGoal()) tempSpd = motorSpd;
        else tempSpd = -motorSpd;
        lateral_move(tempSpd);
        betterDelay(pwmDelay_x);
        horizontal = false;
      }
      linearORturn_move(0, 0);
      backToDefault();
      break;

    case 6:
      convDtoDelay();
      Serial.println("!Starting to turn");
      if      (delta_to_goal[2] >= 0) linearORturn_move(motorSpd, -motorSpd);
      else if (delta_to_goal[2] <  0) linearORturn_move(-motorSpd, motorSpd);
      betterDelay(pwmDelay_turn);
      linearORturn_move(0, 0);
      backToDefault();
      break;

    default:
      //Wait for input
      linearORturn_move(0, 0);
      backToDefault();
      rxCommand();
      if (Serial1.available()) Serial1.read();
  }
}

void backToDefault() {
  mode = -1;
  pwmDelay_x = 0;
  pwmDelay_y = 0;
  pwmDelay_diag = 0;
  pwmDelay_turn = 0;
  delta_to_goal[0] = 0;
  delta_to_goal[1] = 0;
  delta_to_goal[2] = 0;
  ReadyToMove[0] = false;
  ReadyToMove[1] = false;
  ReadyToMove[2] = false;
  ReadyToMove[3] = false;
  ALL_RTM = false;
}

void betterDelay(int pwmDelay) {
  double current_mil = millis();
  while ((millis() - current_mil) < pwmDelay) {
    rxCommand();
    String rxData = Serial1.readString();
  }
}

/*
   rxCommand
   case 1~8
   1 = (ALL) move along x axis by input(cm)
   2 = (ALL) move along y axis by input(cm)
   3 = (ALL) stationary turn by input(degree)
   4 = (ALL) assemble according to the input
   5 = (ALL) disassemble, default back to original position
   6 = (SOLO) move along x axis by input(cm)
   7 = (SOLO) move along y axis by input(cm)
   8 = (SOLO) stationary turn by input(degree)
   99 = Force default / immediate stop
*/
void rxCommand() {
  int assemble_mode = 0;
  if (Serial.available()) {
    uint8_t commandMode = Serial.parseInt();
    switch (commandMode) {
      case 1:
        delta_to_goal[0] = Serial.parseInt();
        mode = 4;
        Serial.print("!To Laptop // ");
        Serial.print("received distance : ");
        Serial.println(delta_to_goal[0]);
        clearSerial();
        break;

      case 2:
        delta_to_goal[1] = Serial.parseInt();
        mode = 3;
        Serial.print("!To Laptop // ");
        Serial.print("received distance : ");
        Serial.println(delta_to_goal[1]);
        clearSerial();
        break;

      case 3:
        delta_to_goal[2] = Serial.parseInt();
        mode = 6;
        Serial.print("!To Laptop // ");
        Serial.print("received angle : ");
        Serial.println(delta_to_goal[2]);
        clearSerial();
        break;

      case 4:
        Serial.print("!To Laptop // ");
        Serial.print(" ID : ");
        Serial.print(ACTUAL_ID);

        assemble_mode = Serial.parseInt();
        if (assemble_mode == 1) {
          rectangle = true;
          Serial.println(" R-Assembly Initiated");
          mode = 0;
        } else if (assemble_mode == 2) {
          vertical = true;
          Serial.println(" V-Assembly Initiated");
          mode = 1;
        } else if (assemble_mode == 3) {
          horizontal = true;
          Serial.println(" H-Assembly Initiated");
          mode = 2;
        }
        clearSerial();
        break;

      case 5:
        mode = 5;
        clearSerial();
        break;

      case 6:
        if (Serial.parseInt() == ACTUAL_ID) {
          delta_to_goal[0] = Serial.parseFloat();
          Serial.print("!To Laptop // ");
          Serial.print(" ID : ");
          Serial.print(ACTUAL_ID);
          Serial.println(" Solo Linear Move");
          mode = 4;
        }
        clearSerial();
        break;

      case 7:
        if (Serial.parseInt() == ACTUAL_ID) {
          delta_to_goal[1] = Serial.parseFloat();
          Serial.print("!To Laptop // ");
          Serial.print(" ID : ");
          Serial.print(ACTUAL_ID);
          Serial.println(" Solo Lateral Move");
          mode = 3;
        }
        clearSerial();
        break;

      case 8:
        if (Serial.parseInt() == ACTUAL_ID) {
          delta_to_goal[2] = Serial.parseFloat();
          Serial.print("!To Laptop // ");
          Serial.print(" ID : ");
          Serial.print(ACTUAL_ID);
          Serial.println(" Solo Stationary Turn");
          mode = 6;
        }
        clearSerial();
        break;

      case 9:
      // TESTING PURPOSE : forces robot to move diag
      // command/ID/direction1234/delayMS
        if (Serial.parseInt() == ACTUAL_ID) {
          int diagDir = Serial.parseInt();
          pwmDelay_diag = Serial.parseInt();
          if (diagDir == 1) diag_move(-motorSpd, 1);
          else if (diagDir == 2) diag_move(motorSpd, 1);
          else if (diagDir == 3) diag_move(motorSpd, 0);
          else if (diagDir == 4) diag_move(-motorSpd, 0);

          betterDelay(pwmDelay_diag);
          linearORturn_move(0,0);
        }
        break;
      case 55:
        ReadyToMove[0] = true;
        clearSerial();
        break;
      case 66:
        ReadyToMove[1] = true;
        clearSerial();
        break;
      case 77:
        ReadyToMove[2] = true;
        clearSerial();
        break;
      case 88:
        ReadyToMove[3] = true;
        clearSerial();
        break;

      case 99:
        mode = -1;
        Serial.print("!To Laptop // ");
        Serial.print(" ID : ");
        Serial.print(ACTUAL_ID);
        Serial.println(" reset complete");
        rectangle = false;
        vertical = false;
        horizontal = false;
        magnet(0,1);
        magnet(1,1);
        clearSerial();
        break;

    }
  }
}

void clearSerial() {
  while (Serial.available()) Serial.read();
}

void switchToTag() {
  Serial1.println("AT+anchor_tag=0");
  delay(60);
  Serial1.println("AT+RST");
  delay(60);
  Serial1.println("AT+interval=5");
  delay(60);
  Serial1.println("AT+switchdis=1");
  delay(60);
  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.println(" Switched to TAG");
}

void switchToAnchor() {
  Serial1.println("AT+anchor_tag=1,3");
  delay(60);
  Serial1.println("AT+RST");
  delay(60);
  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.println(" Switched to ANCHOR");
}

void waitForAllDone() {
  switch (ACTUAL_ID) {
    case 0:
      Serial.println(55);
      break;

    case 1:
      Serial.println(66);
      break;

    case 2:
      Serial.println(77);
      break;

    case 3:
      Serial.println(88);
      break;
  }
  ReadyToMove[ACTUAL_ID] = true;
  while (!ALL_RTM) {
    rxCommand();
    if (ReadyToMove[0] && ReadyToMove[1] && ReadyToMove[2] && ReadyToMove[3]) {
      ALL_RTM = true;
      Serial.println("!All RTM");
    }
  }
}

void getDistance() {
  if (Serial1.available()) {
    char tempChar = Serial1.read();

    if (isDistanceDataReceived) {
      // Process the distance data
      if (tempChar == '\n') {
        // End of distance data
        isDistanceDataReceived = false;

        // Convert the distance string to a float
        distanceValue = distanceString.toFloat();

        // Use the distance value as needed
        distance[distanceIndex] = distanceValue;

        // Clear the distance string for the next value
        distanceString = "";

        if (rxCheck[0] && rxCheck[1] && rxCheck[2]) {
          allDistanceReceived = true;
          rxCheck[0] = false;
          rxCheck[1] = false;
          rxCheck[2] = false;
        } else {
          allDistanceReceived = false;
        }
      } else {
        // Append the character to the distance string
        if (tempChar != 'm') {
          distanceString += tempChar;
        }
      }
    } else {
      // Check if the distance data is starting
      if (tempChar == 'd') {
        int index = Serial1.parseInt();   //Read distance ID
        if (index >= 0 && index <= 2) {    //If ID 0~2 Inclusive
          Serial1.read();                 //Get rid of ':'
          distanceIndex = index;          //save current distance index
          rxCheck[index] = true;          //check rx
          isDistanceDataReceived = true;
        }
      }
    }
  }
}

/*
   Saves current position
   Use current_pos_interesect[0] for x, [1] for y
*/

void trilateration() {
  double x1 = anchor1[0], y1 = anchor1[1];
  double x2 = anchor2[0], y2 = anchor2[1];
  double x3 = anchor3[0], y3 = anchor3[1];

  double d1 = distance[0];
  double d2 = distance[1];
  double d3 = distance[2];

  double A = 2 * (x2 - x1);
  double B = 2 * (y2 - y1);
  double D = 2 * (x3 - x2);
  double E = 2 * (y3 - y2);

  double C = pow(x2, 2) - pow(x1, 2) + pow(y2, 2) - pow(y1, 2) + pow(d1, 2) - pow(d2, 2);
  double F = pow(x3, 2) - pow(x2, 2) + pow(y3, 2) - pow(y2, 2) + pow(d2, 2) - pow(d3, 2);

  double x = (C * E - F * B) / (E * A - B * D);
  double y = (C * D - A * F) / (B * D - A * E);

  current_pos_tril[0] = x;
  current_pos_tril[1] = y;

  current_pos_intersect[0] = ((pow(anchor2_X, 2) - pow(d2, 2) + pow(d1, 2)) / (2 * anchor2_X)) * 100;
  current_pos_intersect[1] = ((pow(anchor3_Y, 2) - pow(d3, 2) + pow(d1, 2)) / (2 * anchor3_Y)) * 100;

  //'!' is to differentiate debug serial communication from commands
  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.print(" x : ");
  Serial.print(current_pos_intersect[0]);
  Serial.print(" y : ");
  Serial.println(current_pos_intersect[1]);
}
