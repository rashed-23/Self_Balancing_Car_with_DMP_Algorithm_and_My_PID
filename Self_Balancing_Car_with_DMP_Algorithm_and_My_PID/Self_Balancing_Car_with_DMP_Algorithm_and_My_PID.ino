#include "I2Cdev.h"                    
#include "MPU6050_6Axis_MotionApps20.h"  //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <Wire.h>
MPU6050 mpu;

// Motor Control Pins
#define IN1 4
#define IN2 33
#define IN3 32
#define IN4 5
#define ENA 18  // PWM for left motor
#define ENB 19  // PWM for right motor


// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
/*********Tune these 4 values for your BOT*********/
double setpoint = 272;  //set the value when the bot is perpendicular to ground using serial monitor.
double sample_time = 10.0;

double input, output;

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

void setup() {
  Serial.begin(115200);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(117);
  mpu.setYGyroOffset(6);
  mpu.setZGyroOffset(-48);
  mpu.setZAccelOffset(1226);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    // //setup PID
    // pid.SetMode(AUTOMATIC);
    // pid.SetSampleTime(10);
    // pid.SetOutputLimits(-255, 255);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

////...........................PID Control..................................

float kp = 20, ki = 200, kd = 1.2;  // best kp=20, ki=200 and kd=1.2

float feedback;
float error, pre_error = 0;
float P, I = 0, D;
unsigned long time1 = 0;
unsigned long time2;
float PID_out;
void PID(double ref) {
  feedback = input;
  error = ref - feedback;
  //if (abs(error) < 5) error = 0;
  P = kp * error;
  time2 = millis();
  float dt = (time2 - time1);
  if (dt < sample_time) return; // sample time
  dt = dt * 1.0 / 1000.0;
  float derivative = (error - pre_error) * 1.0 / dt;
  D = derivative * kd;
  I += error * dt;
  if (I > 0)
    I = min((float)(255 * 1.0 / ki), I);
  else
    I = max((float)(-255 * 1.0 / ki), I);
  PID_out = P + I * ki + D;
  PID_out = constrain(PID_out, -255, 255);
  output=PID_out;
  //int motor_speed = abs(PID_out);

  // Serial.print("DT = ");
  // Serial.println(dt);
  //Serial.print("Pitch:");
  //Serial.print(feedback + 180);
  // Serial.print("P = ");
  // Serial.println(P);
  // Serial.print("I = ");
  // Serial.println(I);
  // Serial.print("D = ");
  // Serial.println(D);
  //Serial.print(',');
  //Serial.print("PID Out:");
 // Serial.println(PID_out);

  // Serial.print("Ref_Angle:");
  // Serial.print(ref);
  // Serial.print("\t");
  // Serial.print("Kalman_Filtered_Angle:");
  // Serial.print(kalmanPitch);
  // Serial.print("\t");
  // Serial.print("My_Filtered_Angle:");
  // Serial.print(myPitch);
  // Serial.print("\t");
  // Serial.print("Without_Filter:");
  // Serial.println(pitch);

  // analogWrite(ENA, motor_speed);
  // analogWrite(ENB, motor_speed);

  pre_error = error;
  time1 = time2;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  //no mpu data - performing PID calculations and output to motors

  //pid.Compute();
  PID(setpoint);
  
  if (input > 250 && input < 290) {  //If the Bot is falling
    if (output < 0)                  //Falling towards front
    {
      analogWrite(ENA, -1 * output);
      analogWrite(ENB, -1 * output);
      Forward();
    }                     //Rotate the wheels forward
    else if (output > 0)  //Falling towards back
    {
      analogWrite(ENA, output);
      analogWrite(ENB, output);
      Reverse();
    }     //Rotate the wheels backward
  } else  //If Bot not falling
    Stop();

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);       //get value for q
    mpu.dmpGetGravity(&gravity, &q);            //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //get value for ypr
    input = ypr[1] * 180 / M_PI + 180; 

    // Print the values in the seriakl monitor and plot graph in the serial ploter.
    Serial.print("Pitch:");
    Serial.print(input);
    Serial.print('\t');
    Serial.print("Output:");
    Serial.println(output);
  }
}
