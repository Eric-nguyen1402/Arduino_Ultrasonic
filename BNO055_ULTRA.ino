// Connect GND, S1 and SR pins together.

#include <Wire.h>

// Define Trig and Echo pin:
#define trigPin 6
#define echoPin 7

float Yaw,Roll,Pitch,magx,magy,magz,accx, accy,accz, gyrox,gyroy,gyroz,q0,q1,q2,q3,Roll2,Pitch2,Yaw2,LIAx,LIAy,LIAz,GRVx,GRVy,GRVz;
const int GY_955=0x29;
String SRoll, SPitch, SYaw, SDis, all;
// Define variables:
long duration;
int distance;
String incomingByte = "";
int count = 0;
String data = "";

void setup() 
{
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Wire.begin();
Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
delay(100);
Wire.beginTransmission(GY_955);
Wire.write(0x3E); // Power Mode 
Wire.write(0x00); // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
Wire.endTransmission();
delay(100);
Wire.beginTransmission(GY_955);
Wire.write(0x3D); // Operation Mode
Wire.write(0x0C); //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
Wire.endTransmission();
delay(100);
Serial.begin(115200);  //Setting the baudrate
delay(100);
}
void loop()
{
  if(Serial.available()){         //From RPi to Arduino
    data = Serial.readStringUntil('\n');
    if( data == "zero"){
      Wire.beginTransmission(GY_955);
      Wire.write(0x08);  
      Wire.endTransmission(false);
      Wire.requestFrom(GY_955,32,true);

    }// end of if check zero 

    if( data == "get"){
      serialEvent();
      // Clear the trigPin by setting it LOW:
      digitalWrite(trigPin, LOW);
      
      delayMicroseconds(2);
    
     // Trigger the sensor by setting the trigPin high for 10 microseconds:
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(20);
      digitalWrite(trigPin, LOW);
      
      // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
      duration = pulseIn(echoPin, HIGH);
      
      // Calculate the distance:
      distance = duration*0.034/2;
      
      // Print the distance on the Serial Monitor (Ctrl+Shift+M):
      if(String(distance).length() >=2 && String(distance).length() < 3){
        SDis = "0" + String(distance);
      }
      else if(String(distance).length() < 2){
        SDis = "000";
      }
      else SDis = String(distance);
      
      all = SDis + "," + SRoll + "," + SPitch + "," + SYaw;
      Serial.println(all);
    }// end of if check get data 
  }
}
void serialEvent ()
{
  Wire.beginTransmission(GY_955);
  Wire.write(0x08);  
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955,32,true);
  // Accelerometer
  accx=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accy=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accz=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  // Magnetometer
  magx=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT
  magy=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT
  magz=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT
  // Gyroscope
  gyrox=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroy=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroz=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  // Euler Angles
  Yaw=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
  Roll=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
  Pitch=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
  // Quaternions
  q0=(int16_t)(Wire.read()|Wire.read()<<8 )/(pow(2,14)); //unit less
  q1=(int16_t)(Wire.read()|Wire.read()<<8 )/(pow(2,14)); //unit less
  q2=(int16_t)(Wire.read()|Wire.read()<<8 )/(pow(2,14)); //unit less
  q3=(int16_t)(Wire.read()|Wire.read()<<8 )/(pow(2,14)); //unit less
  //Convert Quaternions to Euler Angles
  Yaw2=(atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2))))*180/PI;
  Roll2=(asin(2*(q0*q2-q3*q1)))*180/PI;
  Pitch2=(atan2(2*(q0*q1+q2*q3),1-2*(pow(q1,2)+pow(q2,2))))*180/PI;
  if(Yaw2 >=0){
    SYaw = "+" + String(Yaw2);  
  }
  else{
    SYaw = String(Yaw2);   
  }
  
  if(Pitch >= 0){
    SPitch = "+" + String(Pitch);
  }
  else{
    SPitch = String(Pitch);
  }
  
  if(Roll >= 0){
    SRoll = "+" + String(Roll);
  }
  else{
    SRoll = String(Roll);
  }
  if ((SYaw.length()) < 6){
    SYaw = SYaw.substring(0,1) + "0" + SYaw.substring(SYaw.length()-4);
  }
  if((SPitch.length()) < 6){
    SPitch = SPitch.substring(0,1) + "0" + SPitch.substring(SPitch.length()-4);
  }
  if((SRoll.length()) < 6){
    SRoll = SRoll.substring(0,1) + "0" + SRoll.substring(SRoll.length()-4);
  }
}
