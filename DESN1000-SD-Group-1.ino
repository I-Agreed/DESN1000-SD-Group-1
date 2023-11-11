#include <TimeOut.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>

#define MOTOR_TWO_POSITIVE 2
#define MOTOR_TWO_NEGATIVE 4
#define MOTOR_TWO_SPEED 3
#define MOTOR_THREE_POSITIVE 6
#define MOTOR_THREE_NEGATIVE 7
#define MOTOR_THREE_SPEED 5
#define MOTOR_ONE_POSITIVE 9
#define MOTOR_ONE_NEGATIVE 8
#define MOTOR_ONE_SPEED 10
#define NUM_MOTORS 3

#define SOLENOID 11
#define SOLENOID_DELAY 500 // 500 millis

#define IR_SENSOR 12

#define SPEED_MULTIPLIER 1

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

TimeOut timeout0;
enum State state;

Pixy2 pixy;

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
    // Serial.print("Motor ");
    // Serial.print(i+1);
    // Serial.print(": ");
    // Serial.println(motorSpeed);
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
  pinMode(LED_BUILTIN, OUTPUT);

  motorOne.positivePin = MOTOR_ONE_POSITIVE;
  motorOne.negativePin = MOTOR_ONE_NEGATIVE;
  motorOne.speedPin = MOTOR_ONE_SPEED;
  motorOne.angle = -PI/6;

  motorTwo.positivePin = MOTOR_TWO_POSITIVE;
  motorTwo.negativePin = MOTOR_TWO_NEGATIVE;
  motorTwo.speedPin = MOTOR_TWO_SPEED;
  motorTwo.angle = PI;

  motorThree.positivePin = MOTOR_THREE_POSITIVE;
  motorThree.negativePin = MOTOR_THREE_NEGATIVE;
  motorThree.speedPin = MOTOR_THREE_SPEED;
  motorThree.angle = PI/6;

  Serial.begin(9600);
  pixy.init();

  state = GOTO_BALL; //change initial state here
}

void loop() {
  timeout0.handler();

  struct Motor motors[NUM_MOTORS];
  motors[0] = motorOne;
  motors[1] = motorTwo;
  motors[2] = motorThree;

  pixy.ccc.getBlocks();

  switch (state) {
    case FIND_BALL:
      break;

    case GOTO_BALL:
    if (pixy.ccc.numBlocks) {
    Serial.print("Pixy detects number of blocks: ");
    Serial.println(pixy.ccc.numBlocks);

      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 1) { // check if signature is tennis ball
          while (digitalRead(IR_SENSOR == 0)) {
            if (pixy.ccc.blocks[i].m_x <= 105) {
              Serial.println(pixy.ccc.blocks[i].m_x);
              moveInDirection(motors, 1, -PI/6);
            }
            else if (pixy.ccc.blocks[i].m_x >= 210) {
              Serial.println("Ball right");
              moveInDirection(motors, 1, PI/6);
            }
            else {
              Serial.println("Ball");
              moveInDirection(motors, 1, 0);
            }
          }
        }
      }
    }
      break;

    case FIND_GOAL:
      break;

    case GOTO_GOAL:
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      while (
        pixy.ccc.blocks[i].m_signature == 2
        && digitalRead(IR_SENSOR == 1)
        && pixy.ccc.blocks[i].m_width > 150 // approximately 50% of screen
      ) {
        moveInDirection(motors, 1, 0);
      }
    }
      break;

    case KICK:
      enableSolenoid();
      break;
    
    case TEST_MOVEMENT:
      moveInDirection(motors, 1, 0);
      break;
    
    case TEST_KICK:
      digitalWrite(SOLENOID, 1);
      digitalWrite(LED_BUILTIN, 1);
      delay(5000);
      digitalWrite(SOLENOID, 0);
      digitalWrite(LED_BUILTIN, 0);
      delay(5000);
      //enableSolenoid();
      //state = END;
      break;
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
