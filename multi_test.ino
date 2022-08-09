#include "Uduino.h"  // Include Uduino library at the top of the sketch
Uduino uduino("IMU");

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250_9Axis_MotionApps41.h>


#define MPU9250_INCLUDE_DMP_MOTIONAPPS41
#define PCA9548A 0x70
void tcaselect(uint8_t i) {

    if(i > 7) return;

    Wire.beginTransmission(PCA9548A);
    Wire.write(1 << i);
    Wire.endTransmission();
}

MPU9250 mpu0;
MPU9250 mpu1;
MPU9250 mpu2;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize0;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount0;     // count of all bytes currently in FIFO
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer0[64]; // FIFO storage buffer
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// Quaternion q1;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {

    Serial.begin(115200);
    Wire.begin();

    while (!Serial);

    // mpu0
    mpu0.initialize();
    devStatus = mpu0.dmpInitialize();
    mpu0.setXGyroOffset(54); //++
    mpu0.setYGyroOffset(-21); //--
    mpu0.setZGyroOffset(5);
    
    // mpu1
    mpu1.initialize();
    devStatus = mpu1.dmpInitialize();
    mpu1.setXGyroOffset(54); //++
    mpu1.setYGyroOffset(-21); //--
    mpu1.setZGyroOffset(5);
    
    // mpu2
    mpu2.initialize();
    devStatus = mpu2.dmpInitialize();
    mpu2.setXGyroOffset(54); //++
    mpu2.setYGyroOffset(-21); //--
    mpu2.setZGyroOffset(5);

    if (devStatus == 0) {
        
        mpu0.setDMPEnabled(true);
        
        mpu1.setDMPEnabled(true);
        
        mpu2.setDMPEnabled(true);
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize0 = mpu0.dmpGetFIFOPacketSize();
        
        packetSize1 = mpu1.dmpGetFIFOPacketSize();
        
        packetSize2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // Error
        Serial.println("Error!");
    }

    tcaselect(0);
    if (mpu0.testConnection()) {

        // Serial.println("Sensor 1 is active");
    }

    tcaselect(1);
    if (mpu1.testConnection()) {

        // Serial.println("Sensor 2 is active");
    }
    
    tcaselect(2);
    if (mpu2.testConnection()) {

        // Serial.println("Sensor 3 is active");
    }

    delay(1000); //

}

void loop() {

    uduino.update();

    if (uduino.isInit()) {
      if (!dmpReady) {
        Serial.println("IMU not connected.");
        delay(15);
        return;
      }

      tcaselect(0);
      int  mpu0IntStatus = mpu0.getIntStatus();
      fifoCount0 = mpu0.getFIFOCount();

      if ((mpu0IntStatus & 0x10) || fifoCount0 == 1024) { // check if overflow
        mpu0.resetFIFO();
      } else if (mpu0IntStatus & 0x02) {
        while (fifoCount0 < packetSize0) fifoCount0 = mpu0.getFIFOCount();

        mpu0.getFIFOBytes(fifoBuffer0, packetSize0);
        fifoCount0 -= packetSize0;

          // Serial.println("----------------Sensor 1------------------");
        SendQuaternion0();
        //SendEuler();
        //SendYawPitchRoll();
        //SendRealAccel();
        //SendWorldAccel();
        // delay(1000);

      }
      
      tcaselect(1);
      int  mpu1IntStatus = mpu1.getIntStatus();
      fifoCount1 = mpu1.getFIFOCount();

      if ((mpu1IntStatus & 0x10) || fifoCount1 == 1024) { // check if overflow
        mpu1.resetFIFO();
      } else if (mpu1IntStatus & 0x02) {
        while (fifoCount1 < packetSize1) fifoCount1 = mpu1.getFIFOCount();

        mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
        fifoCount1 -= packetSize1;

          // Serial.println("----------------Sensor 2------------------");
        SendQuaternion1();
        //SendEuler();
        //SendYawPitchRoll();
        //SendRealAccel();
        //SendWorldAccel();
        // delay(1000);
      }
      
      tcaselect(2);
      int  mpu2IntStatus = mpu2.getIntStatus();
      fifoCount2 = mpu2.getFIFOCount();

      if ((mpu2IntStatus & 0x10) || fifoCount2 == 1024) { // check if overflow
        mpu2.resetFIFO();
      } else if (mpu2IntStatus & 0x02) {
        while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();

        mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
        fifoCount2 -= packetSize2;

          // Serial.println("----------------Sensor 3------------------");
        SendQuaternion2();
        //SendEuler();
        //SendYawPitchRoll();
        //SendRealAccel();
        //SendWorldAccel();
        // delay(1000);
      }

    }

}




////////////////////////////
//  Functions             //
///////////////////////////


void SendQuaternion0() {
  mpu0.dmpGetQuaternion(&q, fifoBuffer0);
  Serial.print("a/");
  Serial.print(q.w, 4); Serial.print("/");
  Serial.print(q.x, 4); Serial.print("/");
  Serial.print(q.y, 4); Serial.print("/");
  Serial.println(q.z, 4);
}

void SendQuaternion1() {
  mpu1.dmpGetQuaternion(&q, fifoBuffer1);
  Serial.print("b/");
  Serial.print(q.w, 4); Serial.print("/");
  Serial.print(q.x, 4); Serial.print("/");
  Serial.print(q.y, 4); Serial.print("/");
  Serial.println(q.z, 4);
}

void SendQuaternion2() {
  mpu2.dmpGetQuaternion(&q, fifoBuffer2);
  Serial.print("c/");
  Serial.print(q.w, 4); Serial.print("/");
  Serial.print(q.x, 4); Serial.print("/");
  Serial.print(q.y, 4); Serial.print("/");
  Serial.println(q.z, 4);
}


/*
void SendEuler() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print(euler[0] * 180 / M_PI); Serial.print("/");
  Serial.print(euler[1] * 180 / M_PI); Serial.print("/");
  Serial.println(euler[2] * 180 / M_PI);
}

void SendYawPitchRoll() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print(ypr[0] * 180 / M_PI); Serial.print("/");
  Serial.print(ypr[1] * 180 / M_PI); Serial.print("/");
  Serial.println(ypr[2] * 180 / M_PI);
}

void SendRealAccel() {
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("a/");
  Serial.print(aaReal.x); Serial.print("/");
  Serial.print(aaReal.y); Serial.print("/");
  Serial.println(aaReal.z);
}

void SendWorldAccel() {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print("a/");
  Serial.print(aaWorld.x); Serial.print("/");
  Serial.print(aaWorld.y); Serial.print("/");
  Serial.println(aaWorld.z);
}
*/
