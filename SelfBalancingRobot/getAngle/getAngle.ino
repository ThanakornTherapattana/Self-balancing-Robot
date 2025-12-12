// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float rateX, rateY, rateZ;
float accX, accY, accZ;
float angleY, angleGyro=0;

float KalmanAngleY=0,KalmanUncertaintyAngleY=2*2;
float Kalman1Doutput[2]={0,0};

uint32_t LoopTimer;

void kalman_1d(float KalmanState, float KalmanUncertainty, 
    float KalmanInput, float KalmanMeasurement){
    
    KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty+0.004*0.004*4*4;

    float KalmanGain=KalmanUncertainty*1/(1*KalmanUncertainty+3*3);

    KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain)*KalmanUncertainty;

    Kalman1Doutput[0]=KalmanState;
    Kalman1Doutput[1]=KalmanUncertainty;
  }

void gyro_signals(void){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  rateX = g.gyro.x*(180/3.142)+10.04;
  rateY = g.gyro.y*(180/3.142)-1.31;
  rateZ = g.gyro.z*(180/3.142)+0.40;

  accX = (a.acceleration.x)/10;
  accY = (a.acceleration.y)/10;
  accZ = (a.acceleration.z)/10;

  angleY = atan(accZ/accX)*(180/3.142);
  angleGyro += rateY*0.004;
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

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.println("");
  delay(100);
  LoopTimer=micros();
}


void loop() {
  gyro_signals();

  kalman_1d(KalmanAngleY, KalmanUncertaintyAngleY, rateY, angleY);
  KalmanAngleY = Kalman1Doutput[0];
  KalmanUncertaintyAngleY = Kalman1Doutput[1];

  /* Get new sensor events with the readings */
  
  //Serial.print("KalmanAngleY:");
  //Serial.print(KalmanAngleY);
  //Serial.print(", ");
  Serial.print("AccAngle:");
  Serial.print(angleY);
  Serial.print(", ");
  Serial.print("GyroAngle:");
  Serial.println(angleGyro);

  while(micros()-LoopTimer < 4000);
  LoopTimer=micros();


}