void calcCurrentPos() {
  while (!allDistanceReceived) getDistance();
  trilateration();
  allDistanceReceived = false;
}

bool LeftOfGoal() {
  if (current_pos_intersect[0] > goal_coords[0]) return false;
  else return true;
}

bool BelowGoal() {
  if (current_pos_intersect[1] > goal_coords[1]) return false;
  else return true;
}

//Calculates delta_to_goal for assembly/disassembly -> need to be implemented, make array for original coords
void calcDelta() {
  if (rectangle) {
    delta_to_goal[0] = rectangle_assemble_coordinates[ACTUAL_ID][0] - current_pos_intersect[0];
    delta_to_goal[1] = rectangle_assemble_coordinates[ACTUAL_ID][1] - current_pos_intersect[1];
    goal_coords[0] = rectangle_assemble_coordinates[ACTUAL_ID][0];
    goal_coords[1] = rectangle_assemble_coordinates[ACTUAL_ID][1];
  } else if (vertical) {
    delta_to_goal[0] = vertical_assemble_coordinates[ACTUAL_ID][0] - current_pos_intersect[0];
    delta_to_goal[1] = vertical_assemble_coordinates[ACTUAL_ID][1] - current_pos_intersect[1];
    goal_coords[0] = vertical_assemble_coordinates[ACTUAL_ID][0];
    goal_coords[1] = vertical_assemble_coordinates[ACTUAL_ID][1];
  } else if (horizontal) {
    delta_to_goal[0] = horizontal_assemble_coordinates[ACTUAL_ID][0] - current_pos_intersect[0];
    delta_to_goal[1] = horizontal_assemble_coordinates[ACTUAL_ID][1] - current_pos_intersect[1];
    goal_coords[0] = horizontal_assemble_coordinates[ACTUAL_ID][0];
    goal_coords[1] = horizontal_assemble_coordinates[ACTUAL_ID][1];
  }

  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.print(" delta_x : ");
  Serial.print(delta_to_goal[0]);
  Serial.print(" delta_y : ");
  Serial.println(delta_to_goal[1]);
}

void convDtoDelay() {
  float diagonal_distance = sqrt(pow(delta_to_goal[0], 2) + pow(delta_to_goal[1], 2));
  pwmDelay_x = abs(delta_to_goal[0] * 66);
  pwmDelay_y = abs(delta_to_goal[1] * 66);
  pwmDelay_diag = abs(diagonal_distance * 66);
  pwmDelay_turn = abs(delta_to_goal[2] * 18);

  Serial.print("!To Laptop // ");
  Serial.print(" ID : ");
  Serial.print(ACTUAL_ID);
  Serial.print(" delay_x : ");
  Serial.print(pwmDelay_x);
  Serial.print(" delay_y : ");
  Serial.print(pwmDelay_y);
  Serial.print(" delay_turn : ");
  Serial.print(pwmDelay_turn);
  Serial.print(" diagonal distance : ");
  Serial.print(diagonal_distance);
  Serial.print(" delay_diag : ");
  Serial.println(pwmDelay_diag);
}
