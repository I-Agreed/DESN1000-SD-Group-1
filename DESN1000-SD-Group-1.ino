#include <TimeOut.h>
#include <Pixy2.h>

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

#define SOLENOID 11
#define SOLENOID_DELAY 500 // 500 millis

#define IR_SENSOR 12

#define SPEED_MULTIPLIER 1

#define GOAL_SIGNATURE 1
#define BALL_SIGNATURE 2

#define PIXY_MAX_X 316
#define PIXY_MAX_Y 208
#define FIND_ACCEPT_ERROR 15

struct Motor {
  int positivePin;
  int negativePin;
  int speedPin;
  double angle; // angle in radians, clockwise from the front
};

enum State {
  FIND_BALL,
  GOTO_BALL,
  FIND_GOAL,
  GOTO_GOAL,
  KICK,
  END,

  TEST_MOVEMENT,
  TEST_KICK,
  TEST_IR
};

struct Motor motorOne;
struct Motor motorTwo;
struct Motor motorThree;

Pixy2 pixy;
TimeOut timeout0;
enum State state;
struct Motor motors[NUM_MOTORS];

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

void enableSolenoid() {
  digitalWrite(SOLENOID, 1);
  timeout0.timeOut(SOLENOID_DELAY, disableSolenoid);

}

void disableSolenoid() {
  digitalWrite(SOLENOID, 0);
}

void turnInDirection(struct Motor motors[NUM_MOTORS], double speed) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotor(motors[i], speed);
  }
}

int getSignatureIndex(int signature) { // returns -1 if not found
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == signature) {
      return i;
      break;
    }
  }
}

void findBall() {
  int index = getSignatureIndex(BALL_SIGNATURE);

  if (index == -1) {
    turnInDirection(motors, 1);
    return;
  }
  // ball is found
  int x = pixy.ccc.blocks[index].m_x;
  int y = pixy.ccc.blocks[index].m_y;

  int midPoint = PIXY_MAX_X/2;

  if (midPoint - FIND_ACCEPT_ERROR > x) {
    // ball is to the left
    turnInDirection(motors, -0.5);
    return;
  } else if (x > midPoint + FIND_ACCEPT_ERROR) {
    // ball is to the right
    turnInDirection(motors, 0.5);
    return;
  }

  // ball is in the centre
  state = GOTO_BALL;
  return;
}

void findGoal() {
  int index = getSignatureIndex(GOAL_SIGNATURE);

  if (index == -1) {
    turnInDirection(motors, 1);
    return;
  }
  // goal is found
  int x = pixy.ccc.blocks[index].m_x;
  int y = pixy.ccc.blocks[index].m_y;

  int midPoint = PIXY_MAX_X/2;

  if (midPoint - FIND_ACCEPT_ERROR > x) {
    // goal is to the left
    turnInDirection(motors, -0.5);
    return;
  } else if (x > midPoint + FIND_ACCEPT_ERROR) {
    // goal is to the right
    turnInDirection(motors, 0.5);
    return;
  }

  // goal is in the centre
  state = GOTO_GOAL;
  return;
}


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
  pinMode(SOLENOID, OUTPUT);
  pinMode(IR_SENSOR, INPUT);

  motorOne.positivePin = MOTOR_ONE_POSITIVE;
  motorOne.negativePin = MOTOR_ONE_NEGATIVE;
  motorOne.speedPin = MOTOR_ONE_SPEED;
  motorOne.angle = PI/6;

  motorTwo.positivePin = MOTOR_TWO_POSITIVE;
  motorTwo.negativePin = MOTOR_TWO_NEGATIVE;
  motorTwo.speedPin = MOTOR_TWO_SPEED;
  motorTwo.angle = PI;

  motorThree.positivePin = MOTOR_THREE_POSITIVE;
  motorThree.negativePin = MOTOR_THREE_NEGATIVE;
  motorThree.speedPin = MOTOR_THREE_SPEED;
  motorThree.angle = -PI/6;

  motors[0] = motorOne;
  motors[1] = motorTwo;
  motors[2] = motorThree;

  Serial.begin(9600);

  state = TEST_MOVEMENT; // change initial state here
}

void loop() {
  timeout0.handler();


  switch (state) {
    case FIND_BALL:
      break;

    case GOTO_BALL:
      break;

    case FIND_GOAL:
      break;

    case GOTO_GOAL:
      break;

    case KICK:
      enableSolenoid();
      break;
    
    case TEST_MOVEMENT:
      moveInDirection(motors, 1, 0);
      break;
    
    case TEST_KICK:
      enableSolenoid();
      state = END;
    
    case TEST_IR:
      int ir = digitalRead(IR_SENSOR);
      Serial.print("IR Sensor: ");
      Serial.println(ir);
      delay(100);
      break;

    default:
      break;
  }
}
