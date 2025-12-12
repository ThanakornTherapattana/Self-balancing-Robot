// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float rateX, rateY, rateZ;

float accX, accY, accZ;

float ratecalibrationX, ratecalibrationY, ratecalibrationZ;
int ratecalibrationNum;


void gyro_signals(void){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  rateX = g.gyro.x*(180/3.142);
  rateY = g.gyro.y*(180/3.142);
  rateZ = g.gyro.z*(180/3.142);

  accX = (a.acceleration.x)/10;
  accY = (a.acceleration.y)/10;
  accZ = (a.acceleration.z)/10;
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

  for(ratecalibrationNum=0; ratecalibrationNum<2000; ratecalibrationNum++){
    gyro_signals();
    ratecalibrationX += rateX;
    ratecalibrationY += rateY;
    ratecalibrationZ += rateZ;
    delay(1);
  }
  ratecalibrationX/=2000;
  ratecalibrationY/=2000;
  ratecalibrationZ/=2000;
  Serial.print("calX=");
  Serial.print(ratecalibrationX);
  Serial.print(", ");
  Serial.print("calY=");
  Serial.print(ratecalibrationY);
  Serial.print(", ");
  Serial.print("calZ=");
  Serial.print(ratecalibrationZ);
  Serial.print(", ");
}


void loop() {

  gyro_signals();
  rateX-= ratecalibrationX;
  rateY-= ratecalibrationY;
  rateZ-= ratecalibrationZ;
  Serial.print("Rotation X: ");
  Serial.print(accX);
  Serial.print(", Y: ");
  Serial.print(accY);
  Serial.print(", Z: ");
  Serial.print(accZ);
  Serial.println("");

  delay(100);
  
}