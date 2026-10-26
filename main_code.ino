// ================= MOTOR PINS =================
#define right_motor_forward 7
#define right_motor_backward 6
#define left_motor_forward 8
#define left_motor_backward 9

#define right_motor_speed 10
#define left_motor_speed 11

// ================= MUX PINS =================
#define S0 5
#define S1 4
#define S2 3
#define S3 2
#define SIG_PIN A0

// ================= SONAR SENSOR =================
#define front_trig A3
#define front_echo A2

#define left_trig A5
#define left_echo A4

// ================= SENSOR SETTINGS =================
#define sensorNumber 8

int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];

// Leftmost sensor = MUX C1
// Rightmost sensor = MUX C8
int muxChannel[sensorNumber] = {1, 2, 3, 4, 5, 6, 7, 8};

int WeightValue[sensorNumber] = {
  10, 20, 30, 40, 50, 60, 70, 80
};

// Manual calibration value
int Reference_ADC[sensorNumber] = {
  400, 400, 400, 400, 400, 400, 400, 400
};

int sumOnSensor = 0;
int sensorWeight = 0;

// ================= PID SETTINGS =================
float line_position = 45;
float center_position = 45;
float error = 0;
float previous_error = 0;
float derivative = 0;

int base_speed = 90;
int kp = 8;
int kd = 70;

// 0 = black line gives higher ADC
// 1 = reverse logic
bool inverseON = 0;

// ================= OBSTACLE SETTINGS =================
int frontObstacleDistance = 12;  // cm
int leftObstacleDistance = 7;    // cm
int leftWallTarget = 5;          // cm

// Slow obstacle speed
int obstacleTurnSpeed = 65;
int obstacleForwardSpeed = 45;

// Turning timing
int firstRightTurnDelay = 250;
int leftReturnTurnDelay = 250;
int forwardAfterLeftDelay = 350;
int finalRightTurnDelay = 230;

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(front_trig, OUTPUT);
  pinMode(front_echo, INPUT);

  pinMode(left_trig, OUTPUT);
  pinMode(left_echo, INPUT);

  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);

  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);

  Serial.println("Straight Line Follower With Updated Obstacle Logic Ready");
}

// ================= LOOP =================
void loop() {
  int frontDistance = readSonar(front_trig, front_echo);

  if (frontDistance > 0 && frontDistance <= frontObstacleDistance) {
    Serial.println("Obstacle Detected");
    avoidObstacleWithLeftSensor();
  }

  lineFollow();
}

// ================= LINE FOLLOW =================
void lineFollow() {
  read_black_line();

  if (sumOnSensor > 0) {
    line_position = (float)sensorWeight / sumOnSensor;

    error = center_position - line_position;
    derivative = error - previous_error;

    int correction = (error * kp) + (derivative * kd);

    int leftSpeed = base_speed - correction;
    int rightSpeed = base_speed + correction;

    previous_error = error;

    motor(leftSpeed, rightSpeed);
  } 
  else {
    // If line lost, move slowly forward
    motor(45, 45);
  }
}

// ================= OBSTACLE AVOIDANCE =================
void avoidObstacleWithLeftSensor() {
  stop();
  delay(200);

  // Step 1: Turn right to leave the line
  motor(obstacleTurnSpeed, -obstacleTurnSpeed);
  delay(firstRightTurnDelay);

  stop();
  delay(100);

  // Step 2: Go beside obstacle.
  // Keep obstacle on left side.
  // When left sensor value becomes high, obstacle has ended.
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {
    int frontDistance = readSonar(front_trig, front_echo);
    int leftDistance = readSonar(left_trig, left_echo);

    Serial.print("Front: ");
    Serial.print(frontDistance);
    Serial.print(" | Left: ");
    Serial.println(leftDistance);

    // If obstacle still in front, turn right a little
    if (frontDistance > 0 && frontDistance <= frontObstacleDistance) {
      motor(60, -60);
      delay(45);
      stop();
      delay(40);
      continue;
    }

    // If left sensor value becomes high, obstacle ended
    if (leftDistance > leftObstacleDistance) {
      Serial.println("Left sensor high: obstacle ended");
      break;
    }

    // Wall-follow using left sensor
    if (leftDistance < leftWallTarget) {
      // Too close to obstacle, move slightly right
      motor(55, 35);
    } 
    else if (leftDistance > leftWallTarget + 2) {
      // Too far from obstacle, move slightly left
      motor(35, 55);
    } 
    else {
      // Good distance, go straight
      motor(obstacleForwardSpeed, obstacleForwardSpeed);
    }
  }

  stop();
  delay(150);

  // Step 3: When left sensor becomes high, turn left
  // This brings robot back toward the original line path
  motor(35, 60);   // soft left, not rotate
  delay(leftReturnTurnDelay);

  stop();
  delay(100);

  // Step 4: Go forward a little
  motor(obstacleForwardSpeed, obstacleForwardSpeed);
  delay(forwardAfterLeftDelay);

  stop();
  delay(100);

  // Step 5: Turn right again to align with the straight line
  motor(60, 35);   // soft right, not rotate
  delay(finalRightTurnDelay);

  stop();
  delay(100);

  // Step 6: Find line again
  findLineAgain();

  previous_error = 0;

  stop();
  delay(100);

  Serial.println("Obstacle Avoidance Done");
}

// ================= FIND LINE AGAIN =================
void findLineAgain() {
  unsigned long startTime = millis();

  while (millis() - startTime < 6000) {
    read_black_line();

    if (sumOnSensor > 0) {
      Serial.println("Line Found Again");
      return;
    }

    // Move forward slowly while slightly right
    motor(50, 45);
  }

  stop();
  Serial.println("Line Not Found");
}

// ================= READ SONAR =================
int readSonar(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return 999;
  }

  int distance = duration * 0.034 / 2;
  return distance;
}

// ================= READ BLACK LINE =================
void read_black_line() {
  sumOnSensor = 0;
  sensorWeight = 0;

  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(muxChannel[i]);
    delayMicroseconds(200);

    sensorADC[i] = analogRead(SIG_PIN);

    if (sensorADC[i] > Reference_ADC[i]) {
      sensorDigital[i] = inverseON == 0 ? 1 : 0;
    } 
    else {
      sensorDigital[i] = inverseON == 0 ? 0 : 1;
    }

    sumOnSensor += sensorDigital[i];
    sensorWeight += sensorDigital[i] * WeightValue[i];
  }
}

// ================= SELECT MUX CHANNEL =================
void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

// ================= STOP =================
void stop() {
  motor(0, 0);
}

// ================= MOTOR FUNCTION =================
void motor(int LPWM, int RPWM) {
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  if (LPWM > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } 
  else if (LPWM < 0) {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  } 
  else {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, LOW);
  }

  if (RPWM > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } 
  else if (RPWM < 0) {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  } 
  else {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, LOW);
  }

  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}
