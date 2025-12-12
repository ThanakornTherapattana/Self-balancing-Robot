// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Motor A connections
float motorA_speed = 0.70;
int enA = 13;  // ENA pin
int in1 = 14;  // IN1 pin
int in2 = 27;  // IN2 pin

// Motor B connections
float motorB_speed = 0.70;
int enB = 12;  // ENB pin
int in3 = 26;  // IN3 pin
int in4 = 25;  // IN4 pin

// PWM properties
const int freq = 15000;        // PWM frequency
const int pwmChannelA = 0;    // PWM channel for motor A
const int pwmChannelB = 1;    // PWM channel for motor B
const int resolution = 8;     // 8-bit resolution (0-255)

//robot state variables
float rateX, rateY, rateZ;
float accX, accY, accZ;
float angleY;

//some kalman stuffs
float KalmanAngleY;
float KalmanUncertaintyAngleY=2*2;
float Kalman1Doutput[2]={0,0};


// variables for PID control
float setpoint = 0;

float kp=130;//110
float ki=500;
float kd=2.2;//2.2

double dt;
const double dt_MIN = 1e-6;
float integral, previous, output=0;
int INTEGRAL_LIMIT = 200;

float speed;
uint32_t LoopTimer, last_time;

float pid(float actual){
  float error = setpoint-actual;
  float Pterm = error*kp;
  integral += error*dt;
  if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  else if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  float Iterm = integral*ki;

  float Dterm = ((error-previous)/dt)*kd;
  previous = error;

  output = Pterm+Iterm+Dterm;
  return output;
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, 
    float KalmanInput, float KalmanMeasurement){
    
    KalmanState+=dt*KalmanInput;
    KalmanUncertainty=KalmanUncertainty+dt*dt*4*4;

    float KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+3*3);

    KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty;

    Kalman1Doutput[0]=KalmanState;
    Kalman1Doutput[1]=KalmanUncertainty;
  }



void gyro_signals(void){
  //this func use to calculate the rate and acceleration for the robot
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  rateX = g.gyro.x*(180/3.142)+10.04;//+10.04
  rateY = g.gyro.y*(180/3.142)-1.31;//-1.31
  rateZ = g.gyro.z*(180/3.142)+0.40;//+0.40

  accX = (a.acceleration.x)/10;
  accY = (a.acceleration.y)/10;
  accZ = ((a.acceleration.z)/10)-0.12;//-0.12

  //raw angle
  angleY = atan2(accZ, accX)*(180/3.142);
}

void motor_forward(){
  ledcWrite(enA, speed*motorA_speed);
  ledcWrite(enB, speed*motorB_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void motor_backward(){
  ledcWrite(enA, speed*motorA_speed);
  ledcWrite(enB, speed*motorB_speed);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void motor_stop(){
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  ledcWrite(enA, 0);
  ledcWrite(enB, 0);
}

void balance(){

  //Calculate Direction to turn motors
  if (KalmanAngleY > setpoint)
  {motor_forward();}
  
  if (KalmanAngleY < setpoint)
  {motor_backward();}
  
  if (KalmanAngleY > 45 || KalmanAngleY < -45)
  {motor_stop();}
}




void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //some variable for mpu6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.println("");
  delay(100);

  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Configure PWM for motor speed control using new API
  ledcAttachChannel(enA, freq, resolution, pwmChannelA);
  ledcAttachChannel(enB, freq, resolution, pwmChannelB);
  
  //stop the motor at the start
  motor_stop();

  //set initial value for mpu
  gyro_signals();
  KalmanAngleY = angleY;

  //initial time
  LoopTimer=micros();
  last_time = LoopTimer;
}


void loop() {
  while(micros()-LoopTimer < 4000);
  last_time = LoopTimer;
  LoopTimer=micros();
  dt = (LoopTimer-last_time)/1000000.0;
  if(dt<dt_MIN) dt = dt_MIN;
  

  gyro_signals();
  kalman_1d(KalmanAngleY, KalmanUncertaintyAngleY, rateY, angleY);
  KalmanAngleY = Kalman1Doutput[0];
  KalmanUncertaintyAngleY = Kalman1Doutput[1];

  /* Get new sensor events with the readings */
  speed = abs(pid(KalmanAngleY));
  speed = constrain(speed, 0, 255);
  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.print(", Angle :");
  Serial.println(KalmanAngleY);

  balance();

  
}