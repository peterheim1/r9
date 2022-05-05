/*last update 5/5/22
 * it works
 * needs calibration 
 * 
 */
#include <Messenger.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Define Variables we'll be connecting to

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  500 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 2500 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MIN_PULSE_WIDTH 1000   //These are the minimum and maximum wavelength values which serve MG 995.
#define MAX_PULSE_WIDTH 2000
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 60
// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

int x,y;
unsigned long previousMillis = 0; 
long interval = 100; 

// EYES
int red =0;
int green = 0;
int blue =0;

int r1,r2,r3,r4,r5,r6,r7;
int h_r, h_p, h_y;
int rp1 = 90 ;
int rp2 = 179 ;
int rp3 = 90 ;
int rp4 = 90 ;
int rp5 = 135 ;
int rp6 = 135 ;
int rp7 = 90 ;

int lp1 = 90 ;
int lp2 = 179 ;
int lp3 = 90 ;
int lp4 = 90 ;
int lp5 = 90 ;
int lp6 = 90 ;
int lp7 = 90 ;
int rgp = 90;
int lgp = 90;

int head_r = 90;
int head_p = 90;
int head_y = 90;

int eyes,mouth,neo;

int pulseWidth(int angle){ //This function calculates servo's motion angle.
int pulse_wide, analog_value;
pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); //This function get angle from 0 to 180 degrees and map from length minimum value to maximum. 
analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
Serial.println(analog_value);
return analog_value; //The value this function returns.
}

void setup() {
  Serial.begin (115200);
  _Messenger.attach(OnMssageCompleted);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  
  ReadSerial();
  unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillis > interval) {
   
    previousMillis = currentMillis;   
  
   ser_print();
   //Serial.print(Setpoint);
    //Serial.print("  ");
   // Serial.print(right_front_power);
   // Serial.print("  ");
   // Serial.println(angle_tilt);
  }

}

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

  if (_Messenger.checkString("a"))
  {
    //Set wheel angle
   x = _Messenger.readInt();
   y = _Messenger.readInt();
  
   Serial.print(x);//front right
   Serial.print("\t");
   //Serial.println(y);//front left
   //y = map(y, -90, 90, 1230, 1700);
   //Serial.print("\t");
   Serial.println(y);//front left
   //pwm.setPWM(x, 0, y);
   //setServoPulse(x, y);
   pwm.writeMicroseconds(x, y);
    return;

  } 

  if (_Messenger.checkString("b")) //joint test
  {
    //Set wheel angle
   x = _Messenger.readInt();
   y = _Messenger.readInt();
  
   Serial.print(y);//front right
   Serial.print("\t");
   
   y = map(y, 0, 245, 500, 2314);
   Serial.println(y);//front left
   //pwm.setPWM(x, 0, y);
   //setServoPulse(x, y);
   pwm.writeMicroseconds(x, y);
   
    return;

  } 

  if (_Messenger.checkString("g")) // right gripper
  {
   y = _Messenger.readInt();
   y = map(y, 0, 180, 1000, 2000);
   pwm.writeMicroseconds(15, y);
    return;
  } 

  if (_Messenger.checkString("r"))//right joints
  {

    rp1 = _Messenger.readInt();
    rp2 = _Messenger.readInt();
    rp3 = _Messenger.readInt();
    rp4 = _Messenger.readInt();
    rp5 = _Messenger.readInt();
    rp6 = _Messenger.readInt();
    //rp7 = _Messenger.readInt();
    
    r1 = map(rp1, 0, 180, 400, 2400);
    r2 = map(rp2, 0, 180, 1230, 1700);   
    r3 = map(rp3, 0, 180, 400, 2400);
    r4 = map(rp4, 0, 180, 500, 2500);
    r5 = map(rp5, 0, 270, 500, 2500);
    r6 = map(rp6, 0, 270, 500, 2500);
    //r7 = map(rp7, 0, 180, 500, 2500);;

    
    pwm.writeMicroseconds(9, r1); //  j0
    pwm.writeMicroseconds(10, r2); //  j1
    pwm.writeMicroseconds(11, r3); // j3
    pwm.writeMicroseconds(12, r4); // j4
    pwm.writeMicroseconds(13, r5);
    pwm.writeMicroseconds(14, r6);
    //pwm.writeMicroseconds(14, r7);
    //pwm.writeMicroseconds(15, y); //right gripper
     return; 
  }

  if (_Messenger.checkString("s"))//start pos
  {
    pwm.writeMicroseconds(9, 1500);  // j0
    pwm.writeMicroseconds(10, 2000); // j1
    pwm.writeMicroseconds(11, 1500); // j3
    pwm.writeMicroseconds(12, 1500); // j4
    pwm.writeMicroseconds(13, 1500); // j5
    pwm.writeMicroseconds(14, 1500); // j6
   
    pwm.writeMicroseconds(15, 1500);// right gripper
// head
    pwm.writeMicroseconds(6, 1500); // left servo
    pwm.writeMicroseconds(7, 1500); // right servo
    pwm.writeMicroseconds(8, 1500); // yaw

     // j0 no 9
    pwm.writeMicroseconds(5, 800); // j1
    pwm.writeMicroseconds(4, 1500); // j3
    pwm.writeMicroseconds(3, 1500); // j4
    pwm.writeMicroseconds(2, 1500); // j5
    pwm.writeMicroseconds(1, 1500); // j6

    pwm.writeMicroseconds(0, 1500); // left gripper
    return; 
  }

  if (_Messenger.checkString("e"))//start pos
  {
    red = _Messenger.readInt();
    green = _Messenger.readInt();
    blue = _Messenger.readInt();
    SetColour(red,green,blue);
    return; 
  }

  if (_Messenger.checkString("h"))//start pos
  {
    head_r = _Messenger.readInt();
    head_p = _Messenger.readInt();
    head_y = _Messenger.readInt();
    

    h_r = map(head_r, 0, 180, 500, 2000);
    h_p = map(head_p, 0, 180, 500, 2000);
    h_y = map(head_y, 0, 180, 500, 2000);
    int inv_b5 = map(head_p, 0, 180, 2000, 500);
    pwm.writeMicroseconds(6, h_p + head_r);
    pwm.writeMicroseconds(5, inv_b5 + head_r);
    pwm.writeMicroseconds(7, h_y);

    Serial.print(" b6 ");
    Serial.print(inv_b5); //head roll
    Serial.print(" head pitch  ");
    Serial.print(h_p); //head pitch
    Serial.print(" head yaw  ");
    Serial.println(h_y); //head yaw
    
    return; 
  }
  
  

  
}


void ser_print(){
  
Serial.print("l"); //servo position message
Serial.print("\t");
Serial.print(lp1); //l1
Serial.print("\t");
Serial.print(lp2);  //l2
Serial.print("\t");
Serial.print(lp3);  // l3
Serial.print("\t");
Serial.print(lp4);  //l4
Serial.print("\t");
Serial.print(lp5); // l5
Serial.print("\t");
Serial.print(lp6); //l6 
Serial.print("\t");
//Serial.print(lp7); // l7
//Serial.print("\t");
Serial.print(lgp); //lg
Serial.print("\t");
Serial.print(rp1); // r1
Serial.print("\t");
Serial.print(rp2); // r2
Serial.print("\t");
Serial.print(rp3); //3
Serial.print("\t");
Serial.print(rp4); //r4
Serial.print("\t");
Serial.print(rp5); //r5
Serial.print("\t");
Serial.print(rp6); //r6
Serial.print("\t");
//Serial.print(rp7); //r7
//Serial.print("\t");
Serial.print(rgp); //rg
Serial.print("\t");
Serial.print(head_r); //head roll
Serial.print("\t");
Serial.print(head_p); //head pitch
Serial.print("\t");
Serial.print(head_y); //head yaw
Serial.print("\t");
Serial.print("\n");
/*
Serial.print("r"); // right servo position message
Serial.print("\t");
Serial.print(RJ0);
Serial.print("\t");
Serial.print(RJ1);
Serial.print("\t");
Serial.print(RJ2);
Serial.print("\t");
Serial.print(RJ3);
Serial.print("\t");
Serial.print(RJ4);
Serial.print("\t");
Serial.print(RJ5);
Serial.print("\t");
Serial.print(RJ6);
Serial.print("\t");
Serial.print("\n");

Serial.print("j"); // servo position
Serial.print("\t");
Serial.print(p1);// right_pan_joint
Serial.print("\t");
Serial.print(p2);// right_lift_joint
Serial.print("\t");
Serial.print(p6);// right_gripper_joint
Serial.print("\t");
Serial.print(p3);//left_pan_joint
Serial.print("\t");
Serial.print(p4);//left_lift_joint
Serial.print("\t");
Serial.print(p7);// left rotate_joint
Serial.print("\t");
Serial.print(p5); // left_gripper_joint
Serial.print("\t");
Serial.print("\n");
*/

}

void SetColour(int red, int green, int blue)
{
       
    Wire.beginTransmission (8);
    Wire.write(blue); // respond with message of 2 bytes
    Wire.write(green);
    Wire.write(red);
    Wire.endTransmission ();  
}
