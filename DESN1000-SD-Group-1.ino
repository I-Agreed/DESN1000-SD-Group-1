#define MOTOR_ONE_POSITIVE 1
#define MOTOR_ONE_NEGATIVE 2
#define MOTOR_ONE_SPEED 3
#define MOTOR_TWO_POSITIVE 4
#define MOTOR_TWO_NEGATIVE 5
#define MOTOR_TWO_SPEED 6
#define MOTOR_THREE_POSITIVE 7
#define MOTOR_THREE_NEGATIVE 8
#define MOTOR_THREE_SPEED 9
#define NUM_MOTORS 3

#define SPEED_MULTIPLIER 1

struct Motor {
  int positivePin;
  int negativePin;
  int speedPin;
  double angle; // angle in radians, clockwise from the front
}

void setMotor(struct Motor motor, double speed) {

  // Set the motor direction
  if (speed >= 0) {
    digitalWrite(motor.positivePin, 1);
    digitalWrite(motor.negativePin , 0);
  } else {
    digitalWrite(motor.positivePin, 0);
    digitalWrite(motor.negativePin , 1);
  }

  analogWrite(motor.speedPin, abs(speed) * 255 * SPEED_MULTIPLIER);
}

void moveInDirection(struct Motor motors[NUM_MOTORS], double speed, double angle) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    double motorAngle = motors[i].angle - angle - HALF_PI;
    double motorSpeed = speed * cos(motorAngle);
    setMotor(motors[i], motorSpeed);
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
