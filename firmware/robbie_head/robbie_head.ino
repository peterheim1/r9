/*last update 5/5/22
 * 
 * 
 * 
 */
#include <Messenger.h>
#include <Wire.h>
// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

// lights
int redPin = 25;
int greenPin = 26;
int bluePin = 27;
int red;
int green;
int blue;

int mouth_1_Pin = 0;
int mouth_2_Pin = 0;
int mouth_3_Pin = 0;
int mouth_4_Pin = 0;
int mouth_5_Pin = 0;

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
void setup() {
  Serial.begin (115200);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(mouth_1_Pin, OUTPUT);
  pinMode(mouth_2_Pin, OUTPUT);
  pinMode(mouth_3_Pin, OUTPUT);
  pinMode(mouth_4_Pin, OUTPUT);
  pinMode(mouth_5_Pin, OUTPUT);
  

}

void loop() {
  ReadSerial();

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

  if (_Messenger.checkString("e"))
  {
    Serial.println("eye clour");
    red = _Messenger.readInt();
    green = _Messenger.readInt();
    blue = _Messenger.readInt();

    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);  
    
    return;

  }
  if (_Messenger.checkString("m"))
  {
    //mouth setting
  bool m_state = _Messenger.readInt();
    Serial.println(" mouth setting");
    if (m_state ==1){
      Serial.println(" mouth on");
    }
    if (m_state ==0){
      Serial.println(" mouth off");
    }  
    return;

  }
 

}
