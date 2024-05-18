#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 imu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

const int MPU = 0x68; // MPU6050 I2C address
float accx, accy, accz, gyrox, gyroy, gyroz;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int readings = 0;
int incomingByte = 0;
int start=0;
//INICIALIZAÇÃO FILTRO DE KALMAN
int A=1;
int B=0;
int C=0;
int D=0;
int u=0;
int H=1;
float Q=0.005;
float R=0.1;
int I=1;
float Xant_r=0;
float Pant_r=1;
float Xant_p=0;
float Pant_p=1;

void setup() {

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);

  imu.initialize();
  
  
  delay(200);
  /*
  delay(1000);
  Serial.println(1);
  delay(1000);
  Serial.println(2);
  delay(1000);
  Serial.println(3);
  delay(1000);
  */
}
void loop() {
 //READ DATA
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accx=ax/8192.0;
  accy=ay/8192.0;
  accz=az/8192.0;
  gyrox=gx/131.0;
  gyroy=gy/131.0;
  gyroz=gz/131.0;
  

  accAngleX = (atan(accy / sqrt(pow(accx, 2) + pow(accz, 2))) * 180 / PI) - AccErrorX; // See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * accx / sqrt(pow(accy, 2) + pow(accz, 2))) * 180 / PI) - AccErrorY; 
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  
  
  gyrox = gyrox - GyroErrorX; // GyroErrorX ~(-0.56)
  gyroy = gyroy - GyroErrorY; // GyroErrorY ~(2)
  gyroz = gyroz - GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyrox * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyroy * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.90 * gyroAngleX + 0.10 * accAngleX;
  pitch = 0.90 * gyroAngleY + 0.10 * accAngleY;
  float rollKalman=FiltroKalman_Roll(roll);
  float pitchKalman=FiltroKalman_Pitch(pitch);
  
  if (Serial.available() > 0) {

    incomingByte = Serial.read(); // read the incoming byte:
    
    if (incomingByte==97){
      calculate_IMU_error();
      Serial.println("1"); //indica que acabou a calibração
      //Serial.println(incomingByte);
    }
    if (incomingByte==105){
      start=1;
    }
    
    

  }
  // Print the values on the serial monitor
  if(start){
      Serial.print(rollKalman);
      Serial.print(",");
      Serial.println(pitchKalman);
  }
  /*
  Serial.print(rollKalman);
  Serial.print(",");
  Serial.println(pitchKalman);
  delay(100);*/

  /*
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(rollKalman);
  Serial.print(" ");
  Serial.println(pitchKalman);
  delay(150);*/
}
float FiltroKalman_Roll(float medicaoRoll){
  //PREVISAO
  float Xapriori=A*Xant_r+B*u;
  float Papriori=A*Pant_r*A+Q;
  //CORRECAO
  float K=(Papriori*H)/(H*Papriori*H+R);
  float X=Xapriori+K*(medicaoRoll-H*Xapriori);
  float P=(I-K*H)*Papriori;
  Pant_r=P;
  Xant_r=X;
  return X;
}
float FiltroKalman_Pitch(float medicaoPitch){
  //PREVISAO
  float Xapriori=A*Xant_p+B*u;
  float Papriori=A*Pant_p*A+Q;
  //CORRECAO
  float K=(Papriori*H)/(H*Papriori*H+R);
  float X=Xapriori+K*(medicaoPitch-H*Xapriori);
  float P=(I-K*H)*Papriori;
  Pant_p=P;
  Xant_p=X;
  return X;
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read raw values 200 times
  while (readings < 200) {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accx=ax/8192.0;
    accy=ay/8192.0;
    accz=az/8192.0;
    gyrox=gx/131.0;
    gyroy=gy/131.0;
    gyroz=gz/131.0;

  // Sum all readings
    AccErrorX = AccErrorX + ((atan((accy) / sqrt(pow((accx), 2) + pow((accz), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (accx) / sqrt(pow((accy), 2) + pow((accz), 2))) * 180 / PI));
    GyroErrorX = GyroErrorX + gyrox;
    GyroErrorY = GyroErrorY + gyroy;
    GyroErrorZ = GyroErrorZ + gyroz;
    readings++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  readings = 0;
}