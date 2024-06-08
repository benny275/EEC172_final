#include <Stepper.h>

int dt = 500;
float factor = 1.8; // Factor to rotate 1.8 circles

int stepsPerRevolution = 2048;
int motSpeed = 3;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

const int controlPin = 3; // Pin for motor control
int previousControlState = HIGH; // Previous state of control pin
int rotationDirection = -1; // Initial rotation direction is counterclockwise
int rotationCount = 0; // Counter for number of rotations

void setup() {
  Serial.begin(115200);
  myStepper.setSpeed(motSpeed);
  pinMode(controlPin, INPUT); // Set control pin as input
  Serial.println("Stepper motor control ready.");
}

void loop() {
  int controlState = digitalRead(controlPin); // Read the state of the control pin

  // If control pin transitions from HIGH to LOW, change rotation direction
  if (controlState == LOW && previousControlState == HIGH) {
    rotationCount++;
    if (rotationCount % 2 == 0) {
      rotationDirection *= -1; // Change rotation direction every second rotation
    }
  }

  // Calculate the number of steps needed for 1.8 circles
  int steps = int(stepsPerRevolution * factor);

  if (controlState == LOW) {
    if (rotationDirection == 1) {
      Serial.println("Control pin LOW: Rotating clockwise");
      myStepper.step(steps); // Rotate clockwise
    } else {
      Serial.println("Control pin LOW: Rotating counterclockwise");
      myStepper.step(-steps); // Rotate counterclockwise
    }
    delay(dt);
  } else {
    Serial.println("Control pin HIGH: Motor stopped");
    // Optionally, you can add code here to hold the motor in place or stop it completely
  }

  // Update previous control state
  previousControlState = controlState;
}
