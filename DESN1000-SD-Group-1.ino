#define MOTOR_ONE_POSITIVE 2
#define MOTOR_ONE_NEGATIVE 4
#define MOTOR_ONE_SPEED 3
#define MOTOR_TWO_POSITIVE 5
#define MOTOR_TWO_NEGATIVE 6
#define MOTOR_TWO_SPEED 7
#define MOTOR_THREE_POSITIVE 8
#define MOTOR_THREE_NEGATIVE 9
#define MOTOR_THREE_SPEED 10
#define NUM_MOTORS 3

#define SPEED_MULTIPLIER 1

struct Motor {
  int positivePin;
  int negativePin;
  int speedPin;
  double angle; // angle in radians, clockwise from the front
};

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


struct Motor motorOne;
struct Motor motorTwo;
struct Motor motorThree;

void setup() {
  pinMode(MOTOR_ONE_POSITIVE, OUTPUT);
  pinMode(MOTOR_ONE_NEGATIVE, OUTPUT);
  pinMode(MOTOR_ONE_SPEED, OUTPUT);
  pinMode(MOTOR_TWO_POSITIVE, OUTPUT);
  pinMode(MOTOR_TWO_NEGATIVE, OUTPUT);
  pinMode(MOTOR_TWO_SPEED, OUTPUT);
  pinMode(MOTOR_THREE_POSITIVE, OUTPUT);
  pinMode(MOTOR_THREE_NEGATIVE, OUTPUT);
  pinMode(MOTOR_THREE_SPEED, OUTPUT);

  motorOne.positivePin = MOTOR_ONE_POSITIVE;
  motorOne.negativePin = MOTOR_ONE_NEGATIVE;
  motorOne.speedPin = MOTOR_ONE_SPEED;
  motorOne.angle = TWO_PI/3;

  motorTwo.positivePin = MOTOR_TWO_POSITIVE;
  motorTwo.negativePin = MOTOR_TWO_NEGATIVE;
  motorTwo.speedPin = MOTOR_TWO_SPEED;
  motorTwo.angle = TWO_PI/3 * 2;

  motorThree.positivePin = MOTOR_THREE_POSITIVE;
  motorThree.negativePin = MOTOR_THREE_NEGATIVE;
  motorThree.speedPin = MOTOR_THREE_SPEED;
  motorThree.angle = TWO_PI;
}

void loop() {
  struct Motor motors[NUM_MOTORS];
  motors[0] = motorOne;
  motors[1] = motorTwo;
  motors[2] = motorThree;


}
