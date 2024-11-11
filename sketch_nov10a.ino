const int enca[] = {34, 35}; // Encoder A pins for both motors
const int encb[] = {32, 33}; // Encoder B pins for both motors
const int pwm[] = {15, 19};  // PWM pins for motor speed control
const int in1[] = {2, 18};   // Direction pin 1 for both motors
const int in2[] = {4, 5};    // Direction pin 2 for both motors

// Global variables for encoder counts
volatile long pos[] = {0, 0}; 

// Target speed for both motors
const int targetSpeed = 64;

// Error correction parameters
float kp = 0.1; // Proportional gain
float kd = 0.0; // Derivative gain
float ki = 0.0; // Integral gain

// Error storage variables
float ePrev, eIntegral;

long prevT = 0;

void setup() {
  Serial.begin(9600);

  // Initialize pins
  for (int i = 0; i < 2; i++) {
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
    pinMode(pwm[i], OUTPUT);
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
  }

  ePrev = 0.0;
  eIntegral = 0.0;

  // Attach interrupts for the encoders
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder2, RISING);
}

void loop() {

  long currT = micros();
  float deltaT = ((float)(currT-prevT)/(1.0e6));
  prevT = currT; 

  static long prevPos[2] = {0, 0};
  int motorSpeed[2];
  
  // Calculate the speed of both motors
  long currentPos[2];
  noInterrupts(); // Disable interrupts temporarily while reading
  currentPos[0] = pos[0];
  currentPos[1] = pos[1];
  interrupts(); // Re-enable interrupts
  
  long speed1 = (currentPos[0] - prevPos[0])/deltaT;
  long speed2 = (currentPos[1] - prevPos[1])/deltaT;

  // Update previous positions for the next loop iteration
  prevPos[0] = currentPos[0];
  prevPos[1] = currentPos[1];

  // Calculate error
  int error = speed1 - speed2;

  float u = PID(error, deltaT);

  // Adjust motor speeds to synchronize
  motorSpeed[0] = targetSpeed;
  motorSpeed[1] = targetSpeed - u;

  // Ensure the speeds are within the PWM range (0-255)
  motorSpeed[0] = constrain(motorSpeed[0], 0, 255);
  motorSpeed[1] = constrain(motorSpeed[1], 0, 255);

  // Set motor speeds and directions
  setMotor(0, motorSpeed[0], 1);
  setMotor(1, motorSpeed[1], 1);


  Serial.print("Speed1:");
  Serial.print(speed1);
  Serial.print(",");
  Serial.print("Speed2:");
  Serial.println(speed2);

  delay(100);
}

// Function to set motor speed and direction
void setMotor(int motor, int speed, int direction) {
  analogWrite(pwm[motor], speed);
  if (direction == 1) {
    digitalWrite(in1[motor], HIGH);
    digitalWrite(in2[motor], LOW);
  } else {
    digitalWrite(in1[motor], LOW);
    digitalWrite(in2[motor], HIGH);
  }
}

// Encoder interrupt functions
void readEncoder1() {
  int b = digitalRead(encb[0]);
  if (b > 0) pos[0]++;
  else pos[0]--;
}

void readEncoder2() {
  int b = digitalRead(encb[1]);
  if (b > 0) pos[1]++;
  else pos[1]--;
}

float PID(int e, float dt) {

  float dedt = (e-ePrev)/dt;

  eIntegral = eIntegral + e*dt;

  float u = kp*e + kd*dedt + ki*eIntegral;

  ePrev = e;

  return u;
}
