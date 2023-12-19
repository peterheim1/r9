/* last update 20/10/20
 *  teensy 3.5
 *  on gizmo
 *  auto dock works
 *  leds not working
 *  
 */
#include <Arduino.h>
#include <Messenger.h>
#include <PID_v1.h>
#include <Encoder.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//#define OUTPUT_READABLE_QUATERNION
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



Encoder Right_Encoder(32, 31);
Encoder Left_Encoder(36, 35);


long right_count, left_count;

//front right
#define right_front_in1 8
#define right_front_in2 9

//front left
#define left_front_in1 5
#define left_front_in2 4


//speed calc
volatile long right_enc = 0;
volatile long left_enc = 0;
double vel_R, vel_L;

static long countEnc_R = 0;
static long countEnc_L = 0;
float SpeedRight = 0;
float SpeedLeft = 0;

// lights
int redPin = 25;
int greenPin = 26;
int bluePin = 27;

int l_ir = 37;
int r_ir = 38;
int r_bump = 39;

int L_IR_State = 0;
int R_IR_State = 0;
int R_Bump_State = 0;
bool dock = 0;

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
int x = 0;
float v = 0;

int period = 100; //10 hz
unsigned long time_now = 0;

// pid
double Setpoint_R, Input_R, Output_R;
double Setpoint_L, Input_L, Output_L;
PID Left_PID(&Input_L, &Output_L, &Setpoint_L,0.4,1,0.01, DIRECT);
PID Right_PID(&Input_R, &Output_R, &Setpoint_R,0.4,1,0.01, DIRECT);

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();



void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

Serial.begin (115200);


_Messenger.attach(OnMssageCompleted);

analogWriteFrequency(4, 37500);
analogWriteFrequency(5, 37500);
analogWriteFrequency(8, 37500);
analogWriteFrequency(9, 37500);

  //front right
pinMode(right_front_in1, OUTPUT);
pinMode(right_front_in2, OUTPUT);
 //front left
pinMode(left_front_in1, OUTPUT);
pinMode(left_front_in2, OUTPUT);
  // status leds
pinMode(redPin, OUTPUT);
pinMode(greenPin, OUTPUT);
pinMode(bluePin, OUTPUT); 
 
pinMode(l_ir, INPUT);
pinMode(r_ir, INPUT);
pinMode(r_bump, INPUT);


Left_PID.SetMode(AUTOMATIC);
Right_PID.SetMode(AUTOMATIC);

mpu.initialize();
devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 14)..."));
        attachInterrupt(29, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
}
}



void loop() {
  
  ReadSerial();
  //int sensorValue = analogRead(A14);
  
  //v = sensorValue * 0.0110; 
  //v = map(sensorValue, 0, 1023, 0, 128);

//Timing loop
  if(millis() > time_now + period){
        float deltaT = (millis() - time_now)/10;
        time_now = millis();     
        right_enc = Right_Encoder.read();
        left_enc = Left_Encoder.read();
        v = analogRead(A14);
        SpeedRight = ((right_enc - countEnc_R) * 0.029007) * deltaT ;
        Input_R = SpeedRight;
        countEnc_R = right_enc;
        SpeedLeft = ((left_enc - countEnc_L) * 0.029007) * deltaT ; 
        Input_L = SpeedLeft;
        countEnc_L = left_enc;

        Left_PID.Compute();
        Right_PID.Compute();
        Left_PID.SetOutputLimits(-1023, 1023); 
        Right_PID.SetOutputLimits(-1023, 1023);

        Serial.print("o"); // o indicates odometry message
        Serial.print("\t");
        Serial.print(left_enc);
        Serial.print("\t");
        Serial.print(right_enc);
        Serial.print("\t");
        Serial.print(SpeedLeft* 0.001);
        Serial.print("\t");
        Serial.print(SpeedRight* 0.001);
        Serial.print("\t");
        Serial.print(Output_L);
        Serial.print("\t");
        Serial.print(Output_R);
        Serial.print("\t");
        Serial.print("\n");
        
        Serial.print("b"); // o indicates odometry message
        Serial.print("\t");
        Serial.print(v);
        Serial.print("\t");
        Serial.print("\n");
        Serial.print("a");
        Serial.print("\t");
        Serial.print(L_IR_State);
        Serial.print("\t");
        Serial.print(R_IR_State);
        Serial.print("\t");
        Serial.println(R_Bump_State);
        Serial.print("\t");
        Serial.print("\n");
    }//end timing loop


right_front_motor();
left_front_motor();
if (dock > 0)
        AutoDock();
        
if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

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

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("\n");
            Serial.print("q");
            Serial.print("\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.print(q.z);
            Serial.print("\t");
            Serial.print("\n");
            //Serial.print("e");
           // Serial.print("\t");
            //Serial.print("\n");
            Serial.print("a");
            Serial.print("\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\n");
            
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        
    }



}// end main loop

void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{

  if (_Messenger.checkString("d"))
  {
    Serial.println("Diagnostics");
    Serial.print("Encoder Count   Left ");
    Serial.print(Input_L);
    Serial.print("   Right ");
    Serial.println(Input_R);
    Serial.print("power setting   Left ");
    Serial.print(Output_L);
    Serial.print("   Right ");
    Serial.println(Output_R);
    Serial.print("Setpoint  Left ");
    Serial.print(Setpoint_L);
    Serial.print("   Right ");
    Serial.println(Setpoint_R);
    Serial.print("Velocity  Left  ");
    Serial.print(vel_L);
    Serial.print("   Right ");
    Serial.println(vel_R);
    return;

  }
  if (_Messenger.checkString("a"))
  {
    //start auto dock
    dock = 1;
    Serial.println(" starting auto dock");
    //AutoDock();

    return;

  }
  if (_Messenger.checkString("m"))
  {
    //Set wheel angle
    MotorSpeed();

    return;

  }
  if (_Messenger.checkString("e")) //read encoder
  {
   long L = Left_Encoder.read();
   long R = Right_Encoder.read();
   Serial.print(L);
   Serial.print("   ");
   Serial.println(R);
    return;
  }

  if (_Messenger.checkString("c"))// reset encoder
  {
   Left_Encoder.write(0);
   Right_Encoder.write(0);
   
   Serial.println("reset encoders");
    return;
  }

  if (_Messenger.checkString("r"))
  {

    return;
  }

}



void MotorSpeed()
{


  //digitalWrite(MOTOR_PIN, HIGH);
  //lastMotorCommand = millis();
  //motor_power = 1;
  Setpoint_L = _Messenger.readInt();
  Setpoint_R = _Messenger.readInt();



}

void right_front_motor() {
  //Output = Output * -1;
  if (Output_R > 1) {
    analogWrite(right_front_in1, abs(Output_R));
    analogWrite(right_front_in2, 0);
    
  }

  if (Output_R < -1) {
    analogWrite(right_front_in1, 0);
    analogWrite(right_front_in2, abs(Output_R));
    
  }

  if (Output_R == 0) {
    analogWrite(right_front_in1, 0);
    analogWrite(right_front_in2, 0);
    
  }
}

void left_front_motor() {
  //Output = Output * -1;
  if (Output_L > 1) {
    analogWrite(left_front_in1, abs(Output_L));
    analogWrite(left_front_in2, 0);
    
  }

  if (Output_L < -1) {
    analogWrite(left_front_in1, 0);
    analogWrite(left_front_in2, abs(Output_L));
  } 

  if (Output_L == 0) {
    analogWrite(left_front_in1, 0);
    analogWrite(left_front_in2, 0);
    
  }
}

void AutoDock(){
  
  L_IR_State = digitalRead(l_ir);
  R_IR_State = digitalRead(r_ir);
  R_Bump_State = digitalRead(r_bump);
  /*
  Serial.print("left  ");
  Serial.print(L_IR_State);
  Serial.print("  right  ");
  Serial.print(R_IR_State);
  Serial.print("  bump  ");
  Serial.println(R_Bump_State);
  */
  if (R_IR_State < 1 && R_Bump_State == 1){ 
  Setpoint_L = -50; }
  else{Setpoint_L = 0; }
  
  if (L_IR_State < 1 && R_Bump_State == 1){ 
  Setpoint_R = -50; }
  else{Setpoint_R = 0; }

  if (digitalRead(r_bump) == 0){
  
  Serial.print(R_Bump_State);
  Setpoint_L = 0;
  Setpoint_R = 0;  
  dock = 0;
  delay(1);
  }
}
