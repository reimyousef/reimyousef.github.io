/**
 * Dual Stepper Motor Control System
 * ----------------------------------
 * This program controls two stepper motors using the AccelStepper library. 
 * It supports two modes of operation:
 * 
 * - TRAJECTORY mode: Run predefined trajectories (e.g., square or diagonal)
 * - MANUAL mode: Control motors with directional keys (WASD) and adjustable speed presets
 * 
 * Key Features:
 * - User input over Serial
 * - State machine implementation
 * - Preset speed/acceleration profiles
 * - Interrupt handling for manual override
 * - Homing and command fallback support
 * 
 * Dependencies:
 * - AccelStepper library
 * 
 * Author: Reim Yousef (probably stressed)
 * Date: ??? (probably late)
 */

#include <AccelStepper.h>

// Setup for two steppers: driver interface (1), direction+step pins
AccelStepper stepper1(1, 3, 2); 
AccelStepper stepper2(1, 5, 4);

// Enum for system states
enum State {
  COMMAND,
  TRAJECTORY_SELECTION,
  TRAJECTORY_CONTROL,
  MANUAL_SELECTION,
  MANUAL_CONTROL,
  END
};

State currentState = COMMAND;

// System flags and settings
int trajectorySelected = -1;
bool trajectoryInterrupted = false;
bool tuningSpeed = true;
bool running = true;
int xPressCount = 0;

// Presets (speed, acceleration, step size)
const int speedPresets[5] = {50, 100, 150, 200, 250};
const int accelPresets[5] = {25, 50, 75, 100, 125};
const int stepPresets[5] = {10, 20, 30, 40, 50};
int currentPreset = 0;
int stepSize; 
const int delayTime = 1000; // ms

void setup() {
  Serial.begin(9600);
  Serial.println("\nStepper system initializing...");
  
  stepper1.setMaxSpeed(speedPresets[currentPreset]);
  stepper1.setAcceleration(accelPresets[currentPreset]);
  stepper2.setMaxSpeed(speedPresets[currentPreset]);
  stepper2.setAcceleration(accelPresets[currentPreset]);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepSize = stepPresets[currentPreset];

  Serial.println("Stepper ready. Press 'S' to emergency stop.");
  printStateInstructions();
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'S') {
      running = false;
      Serial.println("Emergency stop.");
    }
    if (!running) return;
    handleInput(input);
  }
}

void handleInput(char input) {
  switch (currentState) {
    case COMMAND: handleCommandInput(input); break;
    case TRAJECTORY_SELECTION: handleTrajectorySelectionInput(input); break;
    case MANUAL_SELECTION: handleManualSelectionInput(input); break;
    case MANUAL_CONTROL: handleManualControlInput(input); break;
    case TRAJECTORY_CONTROL:
      runTrajectory();
      currentState = COMMAND;
      Serial.println("Trajectory complete.");
      printStateInstructions();
      break;
    case END:
      Serial.println("System shutdown.");
      break;
  }
}

void handleCommandInput(char input) {
  if (input == 't') currentState = TRAJECTORY_SELECTION;
  else if (input == 'm') currentState = MANUAL_SELECTION;
  else if (input == 'x') {
    xPressCount++;
    if (xPressCount >= 2) currentState = END;
    else Serial.println("Press 'x' again to exit.");
  }
  printStateInstructions();
}

void handleTrajectorySelectionInput(char input) {
  if (input >= '0' && input <= '9') {
    trajectorySelected = input - '0';
    Serial.print("Trajectory "); Serial.print(trajectorySelected);
    Serial.println(" selected. Press 't' to start or 'x' to cancel.");
  } 
  else if (input == 't' && trajectorySelected != -1) {
    currentState = TRAJECTORY_CONTROL;
    printStateInstructions();
  } 
  else if (input == 'x') {
    currentState = COMMAND;
    trajectorySelected = -1;
    Serial.println("Cancelled. Back to Command Mode.");
    printStateInstructions();
  }
}

void handleManualSelectionInput(char input) {
  if (input >= '1' && input <= '5') {
    currentPreset = input - '1';
    stepper1.setMaxSpeed(speedPresets[currentPreset]);
    stepper1.setAcceleration(accelPresets[currentPreset]);
    stepper2.setMaxSpeed(speedPresets[currentPreset]);
    stepper2.setAcceleration(accelPresets[currentPreset]);
    stepSize = stepPresets[currentPreset];

    Serial.print("Preset "); Serial.print(input);
    Serial.print(": Speed="); Serial.print(speedPresets[currentPreset]);
    Serial.print(", Accel="); Serial.println(accelPresets[currentPreset]);
  } 
  else if (input == 'm') {
    currentState = MANUAL_CONTROL;
    printStateInstructions();
  } 
  else if (input == 'x') {
    currentState = COMMAND;
    Serial.println("Back to Command Mode.");
    printStateInstructions();
  }
}

void handleManualControlInput(char input) {
  if (input == 'w') moveSmallSteps('w');
  else if (input == 's') moveSmallSteps('s');
  else if (input == 'a') moveSmallSteps('a');
  else if (input == 'd') moveSmallSteps('d');
  else if (input == 'h') moveHome();
  else if (input == 'x') {
    xPressCount++;
    if (xPressCount >= 2) currentState = END;
    else {
      currentState = COMMAND;
      Serial.println("Back to Command Mode.");
      printStateInstructions();
    }
  }
}

void moveSmallSteps(char direction) {
  int s1 = 0, s2 = 0;

  if (direction == 'w') { s1 = stepSize; s2 = -stepSize; }
  else if (direction == 's') { s1 = -stepSize; s2 = stepSize; }
  else if (direction == 'a') { s1 = -stepSize; s2 = -stepSize; }
  else if (direction == 'd') { s1 = stepSize; s2 = stepSize; }

  stepper1.move(s1);
  stepper2.move(s2);

  while (stepper1.distanceToGo() || stepper2.distanceToGo()) {
    stepper1.run();
    stepper2.run();
    if (checkForInterrupt()) return;
  }

  Serial.println("Movement complete.");
}

void moveHome() {
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  Serial.println("Returning to home position...");

  while (stepper1.currentPosition() || stepper2.currentPosition()) {
    stepper1.run();
    stepper2.run();
  }

  Serial.println("Homing complete.");
}

void runTrajectory() {
  trajectoryInterrupted = false;
  switch (trajectorySelected) {
    case 0: trajectory0_square(); break;
    case 1: trajectory1_diagonal(); break;
    default: Serial.println("Trajectory not implemented."); break;
  }
}

bool checkForInterrupt() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'x') {
      Serial.println("Trajectory interrupted.");
      trajectoryInterrupted = true;
      return true;
    }
  }
  return false;
}

void trajectory0_square() {
  Serial.println("Executing square trajectory.");
  char steps[4] = {'w', 'd', 's', 'a'};
  for (int i = 0; i < 4; i++) {
    moveSmallSteps(steps[i]);
    if (trajectoryInterrupted) return;
    delay(500);
  }
}

void trajectory1_diagonal() {
  Serial.println("Executing diagonal trajectory.");
  stepper1.move(200);
  stepper2.move(200);
  while (stepper1.distanceToGo() || stepper2.distanceToGo()) {
    stepper1.run();
    stepper2.run();
    if (checkForInterrupt()) return;
  }
}

// Placeholder for other trajectories
void trajectory2_dummy() {}
void trajectory3_dummy() {}
void trajectory4_dummy() {}
void trajectory5_dummy() {}
void trajectory6_dummy() {}
void trajectory7_dummy() {}
void trajectory8_dummy() {}
void trajectory9_dummy() {}

void printStateInstructions() {
  Serial.println();
  switch (currentState) {
    case COMMAND:
      Serial.println("[COMMAND MODE]");
      Serial.println("Press 't' to select trajectory, 'm' for manual control, 'x' to exit.");
      break;
    case TRAJECTORY_SELECTION:
      Serial.println("[TRAJECTORY SELECTION] Choose 0–9, 't' to start, 'x' to cancel.");
      break;
    case TRAJECTORY_CONTROL:
      Serial.println("[TRAJECTORY CONTROL] Running... press 'x' to stop.");
      break;
    case MANUAL_SELECTION:
      Serial.println("[MANUAL SELECTION] Choose 1–5 for speed presets, 'm' to begin.");
      break;
    case MANUAL_CONTROL:
      Serial.println("[MANUAL CONTROL] Use 'w', 'a', 's', 'd' to move. 'h' = home, 'x' = exit.");
      break;
    case END:
      Serial.println("[END] System shut down.");
      break;
  }
}