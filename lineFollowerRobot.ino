#define IR_SENSOR_LEFT A0
#define IR_SENSOR_RIGHT A1

#define MOTOR_SPEED 200
#define TURN_SPEED 100

// Motor pins
#define ENA 5  // PWM Speed Control for Motor A
#define IN1 6  // Motor A input 1
#define IN2 7  // Motor A input 2

#define ENB 9  // PWM Speed Control for Motor B
#define IN3 10 // Motor B input 3
#define IN4 11 // Motor B input 4

// PID constants
#define KP 1.2   // Proportional gain
#define KD 0.4   // Derivative gain
#define KI 0.1   // Integral gain

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float pidValue = 0;

void setup() {
  // Motor pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR sensor pins as input
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  int leftSensorValue = analogRead(IR_SENSOR_LEFT);
  int rightSensorValue = analogRead(IR_SENSOR_RIGHT);

  // Calculate error (difference between right and left sensor readings)
  error = rightSensorValue - leftSensorValue;

  // Calculate PID value
  pidValue = (KP * error) + (KD * (error - lastError)) + (KI * integral);

  // Adjust motor speeds based on PID value
  int leftMotorSpeed = MOTOR_SPEED + pidValue;
  int rightMotorSpeed = MOTOR_SPEED - pidValue;

  // Ensure motor speeds are within a valid range (0 to 255)
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Move the robot with adjusted motor speeds
  moveRobot(leftMotorSpeed, rightMotorSpeed);

  // Update PID variables for the next iteration
  lastError = error;
  integral += error;

  // Print sensor values, PID output, and motor speeds for debugging
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(" | Right Sensor: ");
  Serial.print(rightSensorValue);
  Serial.print(" | PID Output: ");
  Serial.print(pidValue);
  Serial.print(" | Left Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightMotorSpeed);

  // Introduce a small delay for stability
  delay(10);
}

void moveRobot(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  digitalWrite(IN1, (leftSpeed > 0) ? HIGH : LOW);
  digitalWrite(IN2, (leftSpeed > 0) ? LOW : HIGH);

  digitalWrite(IN3, (rightSpeed > 0) ? HIGH : LOW);
  digitalWrite(IN4, (rightSpeed > 0) ? LOW : HIGH);
}
