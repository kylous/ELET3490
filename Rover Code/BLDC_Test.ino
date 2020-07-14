#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <ESP32Servo.h>

#define arming 35
#define leftM 32
#define rightM 33
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define RXD2 16       //RX2 - Radio
#define TXD2 17       //TX2 - Radio
#define HC12 Serial2  //Hardware serial 2 on the ESP32 - Radio


Adafruit_BNO055 myIMU = Adafruit_BNO055();
Servo leftMotor;
Servo rightMotor;
int leftSpeed,rightSpeed,rSpeedMap,lSpeedMap,opMode=0, armSpeed, startup = 0;
double heading,relBearing,bearing = -30;
String message;
String command;

uint8_t sys, gyro, accel, mg=0;


void setup() {

  leftMotor.attach(leftM,1000,2000);
  rightMotor.attach(rightM,1000,2000);
  leftMotor.setPeriodHertz(50);      // Standard 50hz servo
  rightMotor.setPeriodHertz(50);      // Standard 50hz servo
  
  Serial.begin(115200);
  HC12.begin(115200, SERIAL_8N1, RXD2, TXD2);       //UART2 - Radio @115200 BAUD
  myIMU.begin();
  myIMU.setExtCrystalUse(true);

  

  
}
void loop() {

  switch(opMode){
    case 0:
    {
      imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      heading = calcHeading(mag.x(),mag.y());  
      myIMU.getCalibration(&sys, &gyro, &accel, &mg);
    
      armSpeed = 0;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);

    
      armSpeed = 50;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);


      armSpeed = 0;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);
      
      message = (String)sys + " , " + (String)mg + " , " + (String)heading + " , " + (String)bearing + " OP: " + opMode + "\r\n";
      sendMessage(message);
      getCommand();

      delay(250);

            
    }
    break;

    case 1:
    {   
      imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

      myIMU.getCalibration(&sys, &gyro, &accel, &mg);
      
      heading = calcHeading(mag.x(),mag.y());
      relBearing = bearing - heading;
      leftSpeed = calcLeftMotor(relBearing);
      rightSpeed = calcRightMotor(relBearing);
    
      Serial.print("Bearing: "); Serial.println(bearing);
      Serial.print("Heading: "); Serial.println(heading);
      Serial.print("Relative Bearing: "); Serial.println(relBearing);
      Serial.print("Left Motor Speed: "); Serial.print(leftSpeed); Serial.print(" || Right Motor Speed: "); Serial.println(rightSpeed);
      Serial.println("-------------------------------------------------");
      rSpeedMap = map(rightSpeed, 0, 255, 0, 180);
      lSpeedMap = map(leftSpeed, 0, 255, 0, 180);

      if(startup == 0){
        for(double start = millis(); millis() - start < 5000;){
          leftMotor.write(0);
          rightMotor.write(0);
        }
        startup = 1;
      }
      
    
      leftMotor.write(lSpeedMap);
      rightMotor.write(rSpeedMap);

      Serial.print(lSpeedMap);Serial.print(" || "); Serial.println(rSpeedMap);
      Serial.println("-------------------------------------------------");

      message = (String)sys + " , " + (String)mg + "|" + (String)heading + "," + (String)bearing + "|" + leftSpeed + "," + rightSpeed +"\r\n";
      sendMessage(message);
      delay(250);
      getCommand();
    }
    break;
  }


}
void getCommand(){
  
  while(HC12.available()>0){
    int incoming = HC12.read();
    if (isDigit(incoming)){
      command += (char)incoming;
    }
  }
  Serial.println(command);

  
  if(command == "1"){
    opMode = 1;
  }
  if(command == "0"){
    opMode = 0;
  }

  command = "";
}


void sendMessage(String message){
  HC12.print(message);
  Serial.print(message);
}

int calcLeftMotor(double relBearing){
  if(relBearing > 0){
    return 200;
  }
  if (relBearing<= -138){
    return 0;
  }
  else{
    return 1.452 * relBearing + 200;
  }
}

int calcRightMotor(double relBearing){
  if(relBearing <= 0){
    return 200;
  }
  if(relBearing >= 138){
    return 0;
  }
  else{
    return -1.452 * relBearing + 200;
  }
}

double calcHeading(int x, int y){
  return (-atan2(x, y) *180 / 3.141592654);
}
