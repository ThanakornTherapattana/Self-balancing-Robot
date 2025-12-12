// Motor A connections
int enA = 13;  // ENA pin
int in1 = 14;  // IN1 pin
int in2 = 27;  // IN2 pin

// Motor B connections
int enB = 12;  // ENB pin
int in3 = 26;  // IN3 pin
int in4 = 25;  // IN4 pin

// PWM properties
const int freq = 15000;        // PWM frequency
const int pwmChannelA = 0;    // PWM channel for motor A
const int pwmChannelB = 1;    // PWM channel for motor B
const int resolution = 8;     // 8-bit resolution (0-255)

void setup() {
  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Configure PWM for motor speed control using new API
  ledcAttachChannel(enA, freq, resolution, pwmChannelA);
  ledcAttachChannel(enB, freq, resolution, pwmChannelB);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set initial PWM duty cycle to 0 (motors off)
  ledcWrite(enA, 0);
  ledcWrite(enB, 0);
}

void loop() {
  directionControl();
  delay(1000);
  speedControl();
  delay(1000);
}

// This function lets you control spinning direction of motors
void directionControl() {
  // Set motors to maximum speed
  ledcWrite(enA, 255);
  ledcWrite(enB, 255);
  
  // Turn on motor A & B (forward direction)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);
  
  // Now change motor directions (reverse direction)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);
  
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors (reverse direction)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  // Accelerate from zero to maximum speed
  for (int i = 200; i < 256; i++) {
    ledcWrite(enA, i);
    ledcWrite(enB, i);
    delay(100);
  }
  
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 200; --i) {
    ledcWrite(enA, i);
    ledcWrite(enB, i);
    delay(100);
  }
  
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}